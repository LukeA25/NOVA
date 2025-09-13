cmake_minimum_required(VERSION 3.16)
project(pi4b C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# External deps
add_library(kissfft STATIC external/kissfft/kiss_fft.c)
target_include_directories(kissfft PUBLIC external/kissfft)

find_package(ALSA REQUIRED)
find_package(SndFile REQUIRED)
find_package(CURL REQUIRED)

# WebRTC VAD (vendored from py-webrtcvad cbits/)
add_library(webrtcvad STATIC
  external/webrtcvad/webrtc/common_audio/vad/vad_core.c
  external/webrtcvad/webrtc/common_audio/vad/vad_filterbank.c
  external/webrtcvad/webrtc/common_audio/vad/vad_gmm.c
  external/webrtcvad/webrtc/common_audio/vad/vad_sp.c
  external/webrtcvad/webrtc/common_audio/vad/webrtc_vad.c
  external/webrtcvad/pywebrtcvad.c
)

target_include_directories(webrtcvad PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/external/webrtcvad
  ${CMAKE_CURRENT_SOURCE_DIR}/external/webrtcvad/webrtc
)

# Library: nova_audio
add_library(nova_audio
  src/audio/doa_gcc_phat.cpp
  src/audio/AlsaCapture.cpp
  src/ServerClient.cpp
)

target_include_directories(nova_audio
  PUBLIC ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(nova_audio
  PRIVATE 
    kissfft
    ALSA::ALSA
    SndFile::sndfile
    CURL::libcurl
    webrtcvad
)

# App
add_executable(app apps/main.cpp)
target_link_libraries(app PRIVATE nova_audio)
