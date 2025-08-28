cmake_minimum_required(VERSION 3.16)
project(pi4b CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# External deps
add_library(kissfft STATIC external/kissfft/kiss_fft.c)
target_include_directories(kissfft PUBLIC external/kissfft)

# ALSA only on Linux
if(APPLE)
  add_compile_definitions(NO_ALSA=1)
else()
  find_package(ALSA REQUIRED)
endif()

# Library: nova_audio
add_library(nova_audio
  src/audio/doa_gcc_phat.cpp
  src/audio/AlsaCapture.cpp
)
target_include_directories(nova_audio
  PUBLIC ${CMAKE_SOURCE_DIR}/include
)
target_link_libraries(nova_audio
  PRIVATE kissfft
)
if(NOT APPLE)
  target_link_libraries(nova_audio PRIVATE ALSA::ALSA)
endif()

# Example/demo
add_executable(doa_demo examples/doa_demo.cpp)
target_link_libraries(doa_demo PRIVATE nova_audio)

# App
add_executable(app apps/main.cpp)
target_link_libraries(app PRIVATE nova_audio)
