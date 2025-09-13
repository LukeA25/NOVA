cmake_minimum_required(VERSION 3.16)
project(pi4b C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# External deps
add_library(kissfft STATIC external/kissfft/kiss_fft.c)
target_include_directories(kissfft PUBLIC external/kissfft)

find_package(ALSA REQUIRED)
find_package(CURL REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SNDFILE REQUIRED sndfile)

# Add webrtcvad static library from all cbits sources
file(GLOB_RECURSE WEBRTCVAD_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/external/webrtcvad/webrtc/common_audio/vad/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/external/webrtcvad/webrtc/common_audio/signal_processing/*.c
)

add_library(webrtcvad STATIC ${WEBRTCVAD_SOURCES})

target_include_directories(webrtcvad PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/external/webrtcvad/webrtc/common_audio/vad/include
    ${CMAKE_CURRENT_SOURCE_DIR}/external/webrtcvad/webrtc/common_audio/signal_processing/include
)

# Library: nova_audio
add_library(nova_audio
  src/audio/doa_gcc_phat.cpp
  src/audio/AlsaCapture.cpp
  src/ServerClient.cpp
)

target_include_directories(nova_audio
  PUBLIC ${CMAKE_SOURCE_DIR}/include
  PRIVATE ${SNDFILE_INCLUDE_DIRS}
)
target_include_directories(webrtcvad PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/external/webrtcvad/webrtc/common_audio/vad/include
)
target_link_libraries(nova_audio
  PRIVATE 
    webrtcvad
    kissfft
    ALSA::ALSA
    CURL::libcurl
    webrtcvad
    ${SNDFILE_LIBRARIES}
)

# App
add_executable(app apps/main.cpp)
target_link_libraries(app PRIVATE nova_audio)
