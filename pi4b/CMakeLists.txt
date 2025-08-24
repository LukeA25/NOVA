cmake_minimum_required(VERSION 3.16)
project(i2s_capture CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(ALSA REQUIRED IMPORTED_TARGET alsa)

add_executable(i2s_capture capture.cpp)
target_link_libraries(i2s_capture PRIVATE PkgConfig::ALSA)
