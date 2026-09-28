# NOVA: Network-Oriented Voice Assistant

NOVA is a robotic desk assistant combining embedded firmware, audio processing, motor control, custom electronics, and a 3D-printed mechanical assembly. The system divides work between a Raspberry Pi 3B, a Pi Zero 2 W, and a Pico W.

![NOVA robotic desk assistant](media/off.png)

## Engineering overview

- **Motor control:** C/FreeRTOS firmware receives binary UART commands and uses separate tasks for servo motion, stepper motion, and head coordination.
- **Voice interaction:** the Pi 3B application captures audio through ALSA, detects speech with WebRTC VAD, and runs Porcupine wake-word detection with Speex resampling.
- **Networked audio:** recorded speech is sent to a remote HTTP service; returned audio is forwarded over UDP to the Pi Zero for playback. Voice responses are not generated entirely on-device.
- **Peripheral control:** the Pi Zero source handles LED states and charging signals through GPIO, alongside UDP audio reception.
- **Hardware design:** the repository includes the mechanical assembly, printable parts, and head PCB design and Gerber files.

## Architecture and source guide

```text
Microphones
    |
    v
Raspberry Pi 3B ---- HTTP ----> Remote audio service
    |                              |
    | <------ response audio ------+
    |
    +---- UART ----> Pico W / FreeRTOS ----> Servos and steppers
    |
    +---- UDP -----> Pi Zero 2 W ----------> Audio playback and LEDs
    ^                    |
    +--- charge events --+
```

- [Pi 3B application](pi3b/apps/main.cpp): audio capture, wake-word handling, voice-service requests, UART commands, and state transitions.
- [Audio components](pi3b/src/audio): ALSA capture, Porcupine integration, and GCC-PHAT direction-of-arrival processing. A [separate example](pi3b/examples/doa_demo.cpp) exercises direction estimation.
- [Pico firmware](pico/src/main.c): UART frame decoding and FreeRTOS motor tasks.
- [Pi Zero application](zero/main.cpp): UDP reception, playback, LEDs, and charge-state handling.
- [Mechanical files](hardware/cad): Fusion 360 and STEP assembly files, plus printable STL parts.
- [PCB files](hardware/pcb): head PCB design and Gerber archive.

## Repository status

This is hardware-specific prototype source. Building and running it requires matching the device configuration to the assembled robot.

The current checkout has known integration gaps:

- `zero/main.cpp` contains unfinished edits in `gpio_thread`, including duplicate declarations, an inconsistent variable name, and stray diff markers. These must be resolved before that target will compile.
- `pi3b/apps/main.cpp` references animation definitions such as `Animation`, `idle_animations`, and `wake_animation` that are not defined in that translation unit or its included project headers.
- Camera-based recognition and object detection are not demonstrated by the current application sources. MQTT is also not implemented in the current applications; communication uses UART, UDP, and HTTP.

The build commands below describe the project layout and configuration. They are not a claim that the complete robot builds or runs from a clean checkout without additional integration work.

## Getting started

### Clone and initialize dependencies

```bash
git clone --recurse-submodules https://github.com/LukeA25/NOVA.git
cd NOVA
```

For an existing checkout:

```bash
git submodule update --init --recursive
```

Run each build section from the repository root.

### Pico W firmware

Requires CMake 3.21 or newer for the checked-in presets, Make, and an ARM GNU bare-metal toolchain. The `picow` preset contains a machine-specific `PICO_TOOLCHAIN_PATH`; override it with the toolchain directory on your machine (the directory containing `bin/arm-none-eabi-gcc`).

```bash
cd pico
cmake --preset picow -DPICO_TOOLCHAIN_PATH=/path/to/arm-none-eabi
cmake --build build
```

The firmware target is `pico_app`. Hold BOOTSEL while connecting the Pico over USB, then copy `pico/build/pico_app.uf2` from the repository to the mounted drive. Review the pin definitions in `pico/src/main.c` against the actual wiring before powering the motors.

### Pi 3B audio application

Build on Raspberry Pi Linux. The current CMake configuration selects the Cortex-A53 **AArch64** Porcupine shared library, so the OS and library architecture must match.

Required development dependencies include CMake, a C/C++ toolchain, pkg-config, ALSA, libcurl, libsndfile, and SpeexDSP, plus the initialized submodules and vendored WebRTC VAD sources.

On a Debian-based Raspberry Pi installation:

```bash
sudo apt update
sudo apt install build-essential cmake git pkg-config libasound2-dev libcurl4-openssl-dev libsndfile1-dev libspeexdsp-dev
cmake -S pi3b -B pi3b/build
cmake --build pi3b/build
```

Before running `./pi3b/build/app`, resolve the integration gaps above and configure the ALSA device, serial device, peer IP addresses, remote audio endpoint, Porcupine model paths, and your own Porcupine access key. These are currently configured in source; they are not environment-variable settings.

The existing `pi3b/setup_deps.sh` also performs a system upgrade and installs optional packages, but omits some dependencies required by CMake. The explicit package list above documents the build dependencies directly.

### Pi Zero peripheral application

The source uses the libgpiod v1 API and invokes `mpg123` for playback. It requires Linux, compatible libgpiod development headers/libraries, and `mpg123`.

After resolving the `gpio_thread` integration gaps:

```bash
cmake -S zero -B zero/build
cmake --build zero/build
```

Configure the GPIO mapping and Pi 3B network address in `zero/main.cpp` before running `./zero/build/app` on the device.
