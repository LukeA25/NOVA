#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include <boards/pico_w.h>
#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <pico/stdlib.h>
#include <pico/cyw43_arch.h>
#include <stdint.h>
#include <stdio.h>

// --- Easing Logic ---
typedef enum {
    EASE_NONE,
    EASE_IN,
    EASE_OUT,
    EASE_IN_OUT,
} EaseMode_t;

static inline float ease_linear(float t)        { return t; }
static inline float ease_in_quad(float t)       { return t * t; }
static inline float ease_out_quad(float t)      { return t * (2.0f - t); }
static inline float ease_in_out_smooth(float t) { return t * t * (3.0f - 2.0f * t); } // 3x^2 - 2x^3

static inline float clamp01(float x) {
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

static inline float select_ease(EaseMode_t e, float t) {
    switch (e) {
        case EASE_IN:      return ease_in_quad(t);
        case EASE_OUT:     return ease_out_quad(t);
        case EASE_IN_OUT:  return ease_in_out_smooth(t);
        case EASE_NONE:
        default:      return ease_linear(t);
    }
}
// ----------------------

// --- Servo Structs --- 
typedef struct {
    float      target_deg;
    float      time_to_complete_ms;
    EaseMode_t ease;
} ServoTarget_t;

typedef struct {
    float         current_deg;
    ServoTarget_t target;
    TaskHandle_t  task;
    uint16_t      pin;
    uint          slice;
} ServoState_t;
// ----------------------


// --- Stepper Structs --- 
typedef struct { float yaw_deg, pitch_deg, roll_deg; } Euler3f;

typedef struct {
    Euler3f    target;
    float      time_to_complete_ms;
    EaseMode_t ease;
} HeadTarget_t;

typedef struct {
  uint8_t in1, in2, in3, in4;
} StepperHW;

typedef struct {
    StepperHW    hw;
    int          dir;             // +1 = forward, -1 = reverse
    uint32_t     step_delay_us;   // delay per half-step
    int          phase;           // 0..3
} Stepper_t;

typedef struct {
    Euler3f           current_rot;
    HeadTarget_t      target;
    TaskHandle_t      task;
    SemaphoreHandle_t lock;
    Stepper_t*        steppers[3];
} HeadState_t;


static const uint8_t FULLSTEP4[4] = { 0b1100, 0b0110, 0b0011, 0b1001 };
// ----------------------


void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint16_t *pulIdleTaskStackSize) {
    static StaticTask_t idleTaskTCB;
    static StackType_t idleTaskStack[configMINIMAL_STACK_SIZE];
    *ppxIdleTaskTCBBuffer = &idleTaskTCB;
    *ppxIdleTaskStackBuffer = idleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint16_t *pulTimerTaskStackSize) {
    static StaticTask_t timerTaskTCB;
    static StackType_t timerTaskStack[configTIMER_TASK_STACK_DEPTH];
    *ppxTimerTaskTCBBuffer = &timerTaskTCB;
    *ppxTimerTaskStackBuffer = timerTaskStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

void vApplicationStackOverflowHook(TaskHandle_t t, char *name){
    printf("STACK OVERFLOW: %s\n", name); for(;;){}
}

void vApplicationMallocFailedHook(void){
    printf("MALLOC FAILED\n"); for(;;){}
}

void servo_set_target(ServoState_t *s, float deg, float time, EaseMode_t ease) {
    ServoTarget_t t = { .target_deg = deg, .time_to_complete_ms = time, .ease = ease };
    s->target = t;
    xTaskNotifyGive(s->task);
}

static inline uint16_t angle_to_us(float deg) {
    if (deg < 0.f)   deg = 0.f;
    if (deg > 270.f) deg = 270.f;
    return (uint16_t)(500.f + (deg / 270.f) * 2000.f);
}

static inline float absf(float x) {
    return x < 0.0f ? -x : x;
}

void servo_task(void *pv) {
    ServoState_t *s = (ServoState_t*)pv;

    // ---- PWM Setup ----
    gpio_set_function(s->pin, GPIO_FUNC_PWM);
    s->slice = pwm_gpio_to_slice_num(s->pin);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 125.0f);
    pwm_init(s->slice, &config, true);
    pwm_set_wrap(s->slice, 20000); // 50 Hz = 20,000 us period
    pwm_set_enabled(s->slice, true);

    const TickType_t step_ms = 10;
    pwm_set_chan_level(s->slice, pwm_gpio_to_channel(s->pin), angle_to_us(s->current_deg));

    for (;;) {
        (void) ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(step_ms));

        // Snapshot the desired target
        ServoTarget_t goal = s->target;
        float start_deg = s->current_deg;
        TickType_t start_tick = xTaskGetTickCount();

        float base_speed = (goal.target_deg - s->current_deg) / goal.time_to_complete_ms;
        uint32_t total_ms = (goal.time_to_complete_ms == 0) ? 1 : goal.time_to_complete_ms;

        TickType_t next_tick = start_tick;
        for (;;) {
            if (ulTaskNotifyTake(pdTRUE, 0) > 0) {
                break; // Break if new target is recieved
            }

            TickType_t now = xTaskGetTickCount();
            uint32_t elapsed_ms = (uint32_t)((now - start_tick) * 1000 / configTICK_RATE_HZ);
            float t = clamp01((float)elapsed_ms / (float)total_ms);

            float E = select_ease(goal.ease, t);
            float deg = start_deg + (goal.target_deg - start_deg) * E;
            s->current_deg = deg;

            // Write PWM
            uint16_t us = angle_to_us(s->current_deg);
            pwm_set_chan_level(s->slice, pwm_gpio_to_channel(s->pin), us);

            if (t >= 1.0f) {
                break; // Break if target is reached
            }

            vTaskDelayUntil(&next_tick, pdMS_TO_TICKS(step_ms));
        }
    }
}

static inline void stepper_write(const StepperHW* h, uint8_t mask){
    gpio_put(h->in1, (mask>>3)&1);
    gpio_put(h->in2, (mask>>2)&1);
    gpio_put(h->in3, (mask>>1)&1);
    gpio_put(h->in4, (mask>>0)&1);
}

void stepper_task(void *pv) {
    Stepper_t *stepper_p = (Stepper_t *)pv;
    StepperHW h = stepper_p->hw;

    // Initialize pins
    gpio_init(h.in1); gpio_set_dir(h.in1, GPIO_OUT);
    gpio_init(h.in2); gpio_set_dir(h.in2, GPIO_OUT);
    gpio_init(h.in3); gpio_set_dir(h.in3, GPIO_OUT);
    gpio_init(h.in4); gpio_set_dir(h.in4, GPIO_OUT);

    // Energize initial phase explicitly
    stepper_p->phase = 0;
    stepper_write(&h, FULLSTEP4[stepper_p->phase]);

    TickType_t period_ticks = pdMS_TO_TICKS(stepper_p->step_delay_us / 1000);
    if (period_ticks == 0) period_ticks = 1;
    TickType_t next_wake = xTaskGetTickCount();

    for (;;) {
        if (stepper_p->dir != 0) {
            // Advance to the next half-step
            stepper_p->phase = (stepper_p->phase + (stepper_p->dir > 0 ? 1 : 3)) & 3;
            stepper_write(&h, FULLSTEP4[stepper_p->phase]);

            period_ticks = pdMS_TO_TICKS(stepper_p->step_delay_us / 1000);
            if (period_ticks == 0) period_ticks = 1;
        }

        vTaskDelayUntil(&next_wake, period_ticks);
    }
}

void head_task(void *pv) {
    HeadState_t *h_p = (HeadState_t *)pv;

    for (;;) {
        (void) ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        HeadTarget_t target = h_p->target;
        TickType_t start = xTaskGetTickCount();
        TickType_t dur   = pdMS_TO_TICKS((uint32_t)target.time_to_complete_ms);
        if (dur == 0) dur = pdMS_TO_TICKS(1);

        // TODO: set each motor's direction from tgt.yaw/pitch/roll mapping
        for (int i = 0; i < 3; ++i) h_p->steppers[i]->dir = +1;

        // Run until duration elapses (or until a *new* command arrives)
        for (TickType_t now = start; (now - start) < dur; now = xTaskGetTickCount()) {
            if (ulTaskNotifyTake(pdTRUE, 0) > 0) {
                target = h_p->target;
                start = now;
                dur   = pdMS_TO_TICKS((uint32_t)target.time_to_complete_ms);
                if (dur == 0) dur = pdMS_TO_TICKS(1);
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        for (int i = 0; i < 3; ++i) h_p->steppers[i]->dir = 0;
    }
}

void stepper_test_task(void *pv) {
    HeadState_t *h_p = (HeadState_t *)pv;

    for (;;) {
        HeadTarget_t test = {.target = {3,3,3}, .time_to_complete_ms = 2000, .ease = EASE_NONE };
        h_p->target = test;
        xTaskNotifyGive(h_p->task);

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void test_task(void *pv) {
    ServoState_t *s = (ServoState_t *)pv;

    for (;;) {
        servo_set_target(s, 50.0f, 4000.0f, EASE_IN_OUT);
        vTaskDelay(pdMS_TO_TICKS(6000));
        servo_set_target(s, 140.0f, 2000.0f, EASE_IN);
        vTaskDelay(pdMS_TO_TICKS(2000));
        servo_set_target(s, 230.0f, 1000.0f, EASE_NONE);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static ServoState_t base_servo;
static ServoState_t mid_servo;
static ServoState_t end_servo;

static HeadState_t head;

int main() {
    stdio_init_all();

    base_servo.pin = 18;
    base_servo.target = (ServoTarget_t){
        .target_deg = 140.0f,
        .time_to_complete_ms = 5000.0f,
        .ease = EASE_NONE
    };

    mid_servo.pin = 19;
    mid_servo.target = (ServoTarget_t){
        .target_deg = 140.0f,
        .time_to_complete_ms = 5000.0f,
        .ease = EASE_NONE
    };

    end_servo.pin = 20;
    end_servo.target = (ServoTarget_t){
        .target_deg = 140.0f,
        .time_to_complete_ms = 5000.0f,
        .ease = EASE_NONE
    };

    static Stepper_t m1 = { 
        .hw = {2,3,4,5},
        .dir = 0, 
        .step_delay_us = 2000, 
        .phase = 0 
    };
    static Stepper_t m2 = {
        .hw = {6,7,8,9},
        .dir = 0,
        .step_delay_us = 2000,
        .phase = 0
    };
    static Stepper_t m3 = {
        .hw = {10,11,12,13},
        .dir = 0,
        .step_delay_us = 2000, 
        .phase = 0 
    };

    head.target = (HeadTarget_t){
        .target = {0,0,0},
        .time_to_complete_ms = 0.0f,
        .ease = EASE_NONE
    };
    head.current_rot = (Euler3f){0,0,0};
    head.lock = xSemaphoreCreateMutex();
    head.steppers[0] = &m1;
    head.steppers[1] = &m2;
    head.steppers[2] = &m3;
    

    // xTaskCreate(servo_task, "Base Servo", 512, &base_servo, 2, &base_servo.task);
    // xTaskCreate(servo_task, "Mid Servo", 512, &mid_servo, 2, &mid_servo.task);
    // xTaskCreate(servo_task, "End Servo", 512, &end_servo, 2, &end_servo.task);

    xTaskCreate(stepper_task, "M1", 256, &m1, 2, NULL);
    xTaskCreate(stepper_task, "M2", 256, &m2, 2, NULL);
    xTaskCreate(stepper_task, "M3", 256, &m3, 2, NULL);

    xTaskCreate(head_task, "Head", 256, &head, 2, &head.task);
    xTaskCreate(stepper_test_task, "Stepper Test", 256, &head, 2, NULL);
    // xTaskCreate(test_task, "Idle", 256, &base_servo, 1, NULL);

   vTaskStartScheduler();

    for (;;) { }
    return 0;
}
