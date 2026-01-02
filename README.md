# 🤖 NOVA: Network-Oriented Voice Assistant

**NOVA** is an open-source robotic desk assistant that combines AI, embedded systems, and animatronics. Designed to be a physical manifestation of a smart assistant, NOVA is capable of tracking, reacting, and interacting with users through voice, vision, and motion, inspired by systems like JARVIS from Iron Man and GLaDOS from the Portal game series.

![NOVA](media/off.png)

---

## ✨ Key Features

- Voice activation with VAD and wake-word detection
- Sound source localization using GCC-PHAT
- Camera-based vision (face tracking, object detection)
- RGB LED “eye” animations
- On-device AI voice responses
- Full 360 degree head rotation with stepper motors in neck
- Battery-powered operation with docking system
- Modular communications (UART, Wi-Fi, I²C, SPI)

---

## 🖥️ System Architecture

### Hardware
- **Raspberry Pi 3B**: Main controller (AI, vision, UI, networking)
- **Pi Zero 2 W**: Vision system, LED eyes, camera
- **Pi Pico W**: Stepper/servo control
- **3D Printed Chassis**: Power + data through rotating joints

### Software
- **Language(s)**: C/C++, Bash
- **Frameworks**: Pico SDK, FreeRTOS, OpenCV
- **Audio Stack**: ALSA, Speex, Porcupine, WebRTC VAD
- **Comms**: UART (serial), I2C, I2S, PWM, WiFi (async MQTT planned)

---

## 🧩 Hardware Design Files

All mechanical and electrical design files are open source and included in the `/hardware/` directory.

- CAD (Fusion 360, STEP, STL): `hardware/cad/`
- PCB layouts and Gerbers: `hardware/pcb/`
- 3D models for integration and fit testing

---

## 🚀 Getting Started


## Build Instructions
#### Clone Repository and Initialize Submodules
```bash
git clone --recurse-submodules https://github.com/your-username/NOVA.git
cd NOVA
```

#### Raspberry Pi Pico Firmware
```bash
cd pico
mkdir build && cd build
cmake .. --preset picow
make
```

#### ⚠️ Flashing the Pico:
1. Hold BOOTSEL while plugging in the Pico.
2. A flash drive will appear. Copy the .uf2 from pico/build/ to the drive. The Pico will reboot and start running the firmware.

#### Raspberry Pi 3B: Audio Processing & AI Backend
```bash
cd pi3b
./setup_deps.sh   # ⬅️ Run this once to install dependencies

mkdir build && cd build
cmake ..
make
./app
```

### Raspberry Pi Zero 2W: Vision and Peripheral Control
```bash
cd zero
mkdir build && cd build
cmake ..
make
./app
```
