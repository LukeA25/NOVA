# 🤖 NOVA: Network-Oriented Voice Assistant

**NOVA** (Network-Oriented Voice Assistant) is an open-source robotic desk assistant that combines AI, embedded systems, and animatronics. Designed to be a physical manifestation of a smart assistant, NOVA is capable of tracking, reacting, and interacting with users through voice, vision, and motion — inspired by systems like J.A.R.V.I.S. and GLaDOS.

![NOVA Animation](media/demo.gif)

---

## 🔧 Project Status

🛠️ **In Development** — This is a long-term robotics and embedded systems project. Actively being designed, coded, and documented.

---

## 🧠 Core Features

- 🎙️ **Voice Activation & VAD**
- 🧭 **Sound Source Localization (GCC-PHAT)**
- 📸 **Camera Vision (Face Tracking, Object Detection)**
- 💡 **LED Eye Animations (RGB)**
- 🔊 **AI Voice Responses via Raspberry Pi**
- 🦾 **Articulated Neck with Stepper/Servo Control**
- 🔋 **Battery Power & Docking System**
- 🌐 **Modular Comms (UART, Wi-Fi, I²C, SPI)**

---

## 🖥️ Architecture Overview

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

## 🚀 Getting Started


## Build Instructions
#### 📦 Clone Repository and Initialize Submodules
```bash
git clone --recurse-submodules https://github.com/your-username/NOVA.git
cd NOVA

# If you already cloned without --recurse-submodules, run:
git submodule update --init --recursive
```

#### 🔌 Raspberry Pi Pico Firmware
```bash
cd pico
mkdir build && cd build
cmake .. --preset picow
make
```

#### ⚠️ Flashing the Pico:
1. Hold BOOTSEL while plugging in the Pico.
2. A flash drive will appear. Copy the .uf2 from pico/build/ to the drive. The Pico will reboot and start running the firmware.

#### 🧠 Raspberry Pi 3B — Audio Processing + AI Backend
```bash
cd pi3b
./setup_deps.sh   # ⬅️ Run this once to install dependencies

mkdir build && cd build
cmake ..
make
./app
```

### 👁 Raspberry Pi Zero 2W — Vision and Peripheral Control
```bash
cd zero
mkdir build && cd build
cmake ..
make
./app
```
