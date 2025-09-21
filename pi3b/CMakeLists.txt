cmake_minimum_required(VERSION 3.16)
project(pi3b C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# -------------------------
# External dependencies
# -------------------------

# kissfft
add_library(kissfft STATIC external/kissfft/kiss_fft.c)
target_include_directories(kissfft PUBLIC external/kissfft)

# ALSA
find_package(ALSA REQUIRED)

# CURL
find_package(CURL REQUIRED)

# libsndfile (via pkg-config)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SNDFILE REQUIRED sndfile)


# -------------------------
# WebRTC VAD (vendored)
# -------------------------

file(GLOB_RECURSE WEBRTCVAD_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/external/webrtcvad/webrtc/common_audio/vad/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/external/webrtcvad/webrtc/common_audio/signal_processing/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/external/webrtcvad/webrtc_stubs.c
)

add_library(webrtcvad STATIC ${WEBRTCVAD_SOURCES})

target_include_directories(webrtcvad PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/external/webrtcvad
    ${CMAKE_CURRENT_SOURCE_DIR}/external/webrtcvad/webrtc/common_audio/vad/include
)

# -------------------------
# Porcupine (vendored)
# -------------------------

add_library(porcupine SHARED IMPORTED)
set_target_properties(porcupine PROPERTIES
    IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/external/porcupine/lib/raspberry-pi/cortex-a53-aarch64/libpv_porcupine.so"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/external/porcupine/include"
)

# -------------------------
# nova_audio library
# -------------------------

add_library(nova_audio
    src/audio/doa_gcc_phat.cpp
    src/audio/AlsaCapture.cpp
    src/audio/PorcupineHandler.cpp
    src/ServerClient.cpp
)

target_include_directories(nova_audio
    PUBLIC ${CMAKE_SOURCE_DIR}/include
    PRIVATE 
    	${SNDFILE_INCLUDE_DIRS}
	${SAMPLERATE_INCLUDE_DIRS}
)

target_link_libraries(nova_audio
    PUBLIC
        webrtcvad
        porcupine
    PRIVATE
	speexdsp
        kissfft
        ALSA::ALSA
        CURL::libcurl
        ${SNDFILE_LIBRARIES}
)

# -------------------------
# Executables
# -------------------------

# Lightweight recording test (no Porcupine, no nova_audio)
add_executable(recording_test tests/recording_test.cpp)
target_link_libraries(recording_test
    PRIVATE
        ALSA::ALSA
        ${SNDFILE_LIBRARIES}
)
target_include_directories(recording_test
    PRIVATE
        ${SNDFILE_INCLUDE_DIRS}
)

# Full app with nova_audio (needs Porcupine etc.)
add_executable(app apps/main.cpp)
target_link_libraries(app PRIVATE nova_audio)
