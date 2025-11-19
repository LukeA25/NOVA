cmake_minimum_required(VERSION 3.13)
project(app LANGUAGES CXX)

# Require C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Create an executable from main.cpp
add_executable(app main.cpp)

# Link pthread for threads
find_package(Threads REQUIRED)
target_link_libraries(app PRIVATE Threads::Threads)

# Link libgpiod if available (Linux only)
if (NOT APPLE)
    find_library(GPIOD_LIB gpiod)
    if (GPIOD_LIB)
        target_link_libraries(app PRIVATE ${GPIOD_LIB})
    endif()
endif()
