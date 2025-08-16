cmake_minimum_required(VERSION 3.13)

# Import Pico SDK
include(pico_sdk_import.cmake)

# Project name and supported languages
project(pico C CXX ASM)

# C/C++ standard versions
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Target RP2040 platform and FreeRTOS OS setting
set(PICO_PLATFORM rp2040)
set(PICO_OS freertos)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialize the SDK
pico_sdk_init()

# Add your FreeRTOS sources manually
add_executable(pico
    main.c

    # Core FreeRTOS
    freertos/FreeRTOS-Kernel/list.c
    freertos/FreeRTOS-Kernel/queue.c
    freertos/FreeRTOS-Kernel/tasks.c
    freertos/FreeRTOS-Kernel/timers.c
    ${CMAKE_CURRENT_LIST_DIR}/freertos/FreeRTOS-Kernel/event_groups.c
    freertos/FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040/port.c
    freertos/FreeRTOS-Kernel/portable/MemMang/heap_4.c
)

set_source_files_properties(main.c PROPERTIES LANGUAGE C)

# Include FreeRTOS and project directories
target_include_directories(pico PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/freertos
    freertos/FreeRTOS-Kernel/include
    freertos/FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040
    freertos/FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040/include
)

# Link against Pico standard libraries
target_link_libraries(pico
    pico_stdlib
    pico_cyw43_arch_none
    hardware_pwm
)

# Generate UF2 and bin files
pico_add_extra_outputs(pico)
