cmake_minimum_required(VERSION 3.13)
project(i2s_capture CXX)
set(CMAKE_CXX_STANDARD 17)
add_executable(i2s_capture capture.cpp)
target_link_libraries(i2s_capture asound)

