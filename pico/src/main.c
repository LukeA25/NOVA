// Copyright (c) 2025 Luke Anderson – MIT License

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include <boards/pico.h>
#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <hardware/uart.h>
#include <pico/stdlib.h>
#include <stdint.h>
#include <stdio.h>

#define UART_ID    uart0
#define BAUD       115200
#define TX_PIN     16
#define RX_PIN     17

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
        default:           return ease_linear(t);
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
typedef struct { float yaw, pitch, roll; } Euler3f;

typedef struct {
    Euler3f    omega;
    uint16_t   time_to_complete_ms;
} HeadTarget_t;

typedef struct {
    uint8_t step_pin;
    uint8_t dir_pin;
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
// ----------------------

// --- UART Structs --- 
typedef struct {
    HeadTarget_t head;
    ServoTarget_t servos[3];
} FullCommand_t;

typedef struct {
    HeadState_t *head;
    ServoState_t *servos[3];
} ControlBundle_t;
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
    printf("MALLOC FAILED\n"); for(;;){} }

static inline uint16_t angle_to_us(float deg) {
    if (deg < 0.f)   deg = 0.f;
    if (deg > 270.f) deg = 270.f;
    return (uint16_t)(500.f + (deg / 270.f) * 2000.f);
}

static inline float absf(float x) {
    return x < 0.0f ? -x : x;
}

static inline float fmaxf(float x, float y) {
    return x > y ? x : y;
}

void compute_motor_speeds(Euler3f omega, float* m_out) {
    float c = 0.7071f;  // ≈ sqrt(2)/2
    float c2 = 0.3535f; // ≈ 0.5 * sqrt(2)/2
    float s2 = 0.6124f; // ≈ sqrt(3)/2 * sqrt(2)/2

    // Compute raw motor "speeds" (steps/sec, not delay yet)
    float raw[3];
    raw[0] = c * omega.yaw + c * omega.pitch;
    raw[1] = c2 * -omega.pitch + s2 * omega.roll + c * omega.yaw;
    raw[2] = c2 * -omega.pitch - s2 * omega.roll + c * omega.yaw;

    // Find max absolute speed
    float max_speed = fmaxf(absf(raw[0]), fmaxf(absf(raw[1]), absf(raw[2])));

    if (max_speed < 1e-3f) {
        m_out[0] = m_out[1] = m_out[2] = INTMAX_MAX;
        return;
    }

    for (int i = 0; i < 3; ++i) {
        if (raw[i] == 0.0f) {
            m_out[i] = 1e6f; // no movement
        } else {
            m_out[i] = max_speed / raw[i] * 1000;
        }
    }
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

void stepper_task(void *pv) {
    Stepper_t *stepper_p = (Stepper_t *)pv;
    StepperHW h = stepper_p->hw;

    // Initialize pins
    gpio_init(h.step_pin); gpio_set_dir(h.step_pin, GPIO_OUT);
    gpio_init(h.dir_pin);  gpio_set_dir(h.dir_pin, GPIO_OUT);

    // Energize initial phase explicitly
    stepper_p->phase = 0;

    TickType_t period_ticks = pdMS_TO_TICKS(stepper_p->step_delay_us / 1000);
    if (period_ticks == 0) period_ticks = 1;
    TickType_t next_wake = xTaskGetTickCount();

    int step = 0;

    for (;;) {
        gpio_put(h.dir_pin, stepper_p->dir > 0 ? 1 : 0);

        if (stepper_p->dir != 0) {
            gpio_put(h.step_pin, step);
            step ^= 1;
            period_ticks = pdMS_TO_TICKS(stepper_p->step_delay_us / 1000);
        } else {
            period_ticks = pdMS_TO_TICKS(1);
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

        float speeds[3];
        compute_motor_speeds(target.omega, speeds);

        for (int i = 0; i < 3; i++) {
            h_p->steppers[i]->dir = (speeds[i] > 0) ? 1 : ((speeds[i] < 0) ? -1 : 0);
            h_p->steppers[i]->step_delay_us = absf(speeds[i]);
        }

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

void uart_command_task(void *pv) {
    ControlBundle_t* bundle_p = (ControlBundle_t*)pv;
    uint8_t buffer[32]; // one extra just in case
    FullCommand_t cmd;

    uart_init(UART_ID, BAUD);
    gpio_set_function(TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(RX_PIN, GPIO_FUNC_UART);

    while (uart_is_readable(UART_ID)) {
        uint8_t dump;
        uart_read_blocking(UART_ID, &dump, 1);
    }

    for (;;) {
        // Find the header first
        uint8_t byte;
        while (1) {
            uart_read_blocking(UART_ID, &byte, 1);
            if (byte == 0xAB) {
                uart_read_blocking(UART_ID, &byte, 1);
                if (byte == 0xCD) {
                    // Header matched, break out
                    break;
                }
            }
        }

        // Header is already found, read the rest of the 29 bytes
        buffer[0] = 0xAB;
        buffer[1] = 0xCD;
        uart_read_blocking(UART_ID, &buffer[2], 29);  // total now 31

        printf("Received header!\n");

        // Continue with parsing...
        cmd.head.omega.pitch = *((float*)&buffer[2]);
        cmd.head.omega.roll  = *((float*)&buffer[6]);
        cmd.head.omega.yaw   = *((float*)&buffer[10]);
        cmd.head.time_to_complete_ms = (uint16_t)buffer[14] | ((uint16_t)buffer[15] << 8);

        for (int i = 0; i < 3; i++) {
            int offset = 16 + i * 5;
            cmd.servos[i].target_deg = (uint16_t)buffer[offset] | ((uint16_t)buffer[offset + 1] << 8);
            cmd.servos[i].time_to_complete_ms = (uint16_t)buffer[offset + 2] | ((uint16_t)buffer[offset + 3] << 8);
            cmd.servos[i].ease = buffer[offset + 4];
        }

        // Debug print
        printf("[UART RX] yaw=%.1f pitch=%.1f roll=%.1f servo0=%.1f\n",
            cmd.head.omega.yaw, cmd.head.omega.pitch, cmd.head.omega.roll,
            cmd.servos[0].target_deg);

        // Notify tasks
        bundle_p->head->target = cmd.head;
        xTaskNotifyGive(bundle_p->head->task);
        for (int i = 0; i < 3; i++) {
            bundle_p->servos[i]->target = cmd.servos[i];
            xTaskNotifyGive(bundle_p->servos[i]->task);
        }
    }
}

void stepper_test_task(void *pv) {
    HeadState_t *h_p = (HeadState_t *)pv;

    HeadTarget_t test = { .omega = {0,0,0}, .time_to_complete_ms = 5000 }; // 9700

    for (;;) {
        test.omega.pitch = 30; test.omega.roll = 0; test.omega.yaw = 0;
        h_p->target = test;
        xTaskNotifyGive(h_p->task);
        vTaskDelay(pdMS_TO_TICKS(10000));

        test.omega.pitch = 0; test.omega.roll = 30; test.omega.yaw = 0;
        h_p->target = test;
        xTaskNotifyGive(h_p->task);
        vTaskDelay(pdMS_TO_TICKS(10000));

        test.omega.pitch = 0; test.omega.roll = 0; test.omega.yaw = 30;
        h_p->target = test;
        xTaskNotifyGive(h_p->task);
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

static ServoState_t base_servo;
static ServoState_t mid_servo;
static ServoState_t end_servo;

static HeadState_t head;

int main() {
    stdio_init_all();

	ServoTarget_t default_target = {
        .target_deg = 140.0f,
        .time_to_complete_ms = 5000.0f,
        .ease = EASE_NONE
    };

    base_servo.pin = 18;
    base_servo.target = default_target;

    mid_servo.pin = 19;
    mid_servo.target = default_target;

    end_servo.pin = 20;
    end_servo.target = default_target;

    static Stepper_t m1 = { 
        .hw = { .step_pin = 3, .dir_pin = 2 },
        .dir = 0, 
        .step_delay_us = 1000,
        .phase = 0 
    };
    static Stepper_t m2 = {
        .hw = { .step_pin = 7, .dir_pin = 6 },
        .dir = 0,
        .step_delay_us = 1000,
        .phase = 0
    };
    static Stepper_t m3 = {
        .hw = { .step_pin = 11, .dir_pin = 10 },
        .dir = 0,
        .step_delay_us = 1000, 
        .phase = 0 
    };

    head.target = (HeadTarget_t){
        .omega = {0,0,0},
        .time_to_complete_ms = 0,
    };
    head.current_rot = (Euler3f){0,0,0};
    head.lock = xSemaphoreCreateMutex();
    head.steppers[0] = &m1;
    head.steppers[1] = &m2;
    head.steppers[2] = &m3;

    static ControlBundle_t bundle = {
        .head = &head,
        .servos = { &base_servo, &mid_servo, &end_servo },
    };
    
    xTaskCreate(uart_command_task, "Idle", 256, &bundle, 1, NULL);

    xTaskCreate(servo_task, "Base Servo", 512, &base_servo, 2, &base_servo.task);
    xTaskCreate(servo_task, "Mid Servo", 512, &mid_servo, 2, &mid_servo.task);
    xTaskCreate(servo_task, "End Servo", 512, &end_servo, 2, &end_servo.task);

    xTaskCreate(stepper_task, "M1", 256, &m1, 2, NULL);
    xTaskCreate(stepper_task, "M2", 256, &m2, 2, NULL);
    xTaskCreate(stepper_task, "M3", 256, &m3, 2, NULL);
    xTaskCreate(head_task, "Head", 256, &head, 3, &head.task);

    vTaskStartScheduler();

    for (;;) { }
    return 0;
}
