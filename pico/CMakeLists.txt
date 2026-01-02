cmake_minimum_required(VERSION 3.13)

# Pull in the Pico SDK
include(${CMAKE_CURRENT_LIST_DIR}/external/pico-sdk/pico_sdk_init.cmake)

project(pico C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Init SDK
pico_sdk_init()

# ---- FreeRTOS locations ----
set(FREERTOS_DIR ${CMAKE_CURRENT_LIST_DIR}/external/FreeRTOS-Kernel)
set(RP2040_PORT  ${FREERTOS_DIR}/portable/ThirdParty/GCC/RP2040)

add_executable(pico_app
    src/main.c
    ${FREERTOS_DIR}/list.c
    ${FREERTOS_DIR}/queue.c
    ${FREERTOS_DIR}/tasks.c
    ${FREERTOS_DIR}/timers.c
    ${FREERTOS_DIR}/event_groups.c
    ${RP2040_PORT}/port.c
    ${FREERTOS_DIR}/portable/MemMang/heap_4.c
)

# Where FreeRTOS headers live (+ your FreeRTOSConfig.h)
target_include_directories(pico_app PRIVATE
    ${FREERTOS_DIR}/include
    ${RP2040_PORT}/include
    ${CMAKE_CURRENT_LIST_DIR}/freertos
)

# Link Pico libs you use
target_link_libraries(pico_app
    pico_stdlib
    hardware_exception
    hardware_pwm
)

# (optional) choose your stdio
# pico_enable_stdio_usb(pico_app 1)
# pico_enable_stdio_uart(pico_app 0)

# UF2/bin/elf outputs
pico_add_extra_outputs(pico_app)
pico_enable_stdio_usb(pico_app 1)
pico_enable_stdio_uart(pico_app 0)
