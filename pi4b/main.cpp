#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include <hardware/gpio.h>

enum class State_t {
    SLEEP,
    IDLE,
    IDLE_ANIMATION,
    SCANNING,
    LISTENING,
    SPEAKING,
};

void initialize_pi() {
    stdio_init_all();
    stdio_uart_init();
    
    // -- UART Setup --
    gpio_set_function(XX, GPIO_FUNC_UART);
    gpio_set_function(XX, GPIO_FUNC_UART);
    uart_set_baudrate(XXX);
    uart_write_blocking(); // Test write to pico
    uart_read_blocking();

    // -- WiFi Setup --
    cyw43_write_bytes(); // Test write to zero
    cyw43_read_bytes();
}

void listening_task(void *pv) {
    State_t *state = (State_t*)pv;

    for (;;) {

    }
}

void idle_task(void *pv) {
    State_t *state = (State_t*)pv;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}


int main() {
    initialize_pi();
    State_t state = State_t::IDLE;

    xTaskCreate(idle_task, "Idle Task", 128, &state, 0, NULL);

    vTaskStartScheduler();

    for (;;) { }
    return 0;

    return 0;
}

/* 
    State_t state = State_t::IDLE;

    for (;;) {
        switch (state) {
            case State_t::IDLE: 
                if (hears wake word) {
                    state = State_t::LISTENING;
                };
            case State_t::IDLE_ANIMATION:
                //something
            case State_t::SCANNING:
                //something
            case State_t::LISTENING:
                //something
            case State_t::SPEAKING:
                if (hears wake word) {
                    state = State_t::LISTENING;
                }
                //something
            case State_t::SLEEP:
            default:
                if (stops charging) {
                    // Wake up procedure
                    state = State_t::IDLE;
                }
                //something
        }
    }
 */
