cmake_minimum_required(VERSION 3.16)
project(pi4b CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# On macOS, we don't have ALSA; build with a stub
if(APPLE)
  add_compile_definitions(NO_ALSA=1)
else()
  find_package(ALSA REQUIRED)
endif()

add_executable(pi4b
  main.cpp
  capture.cpp     # your file in the project root
)

target_include_directories(pi4b PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

if(NOT APPLE)
  target_link_libraries(pi4b PRIVATE ALSA::ALSA)
endif()
