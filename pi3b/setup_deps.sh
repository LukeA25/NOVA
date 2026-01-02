#!/usr/bin/env bash
set -e

echo "🔧 Setting up NOVA dependencies (Pi 3B)..."

# Ensure script is run as root
if [[ "$EUID" -ne 0 ]]; then
  echo "Please run as root: sudo ./setup_deps.sh"
  exit 1
fi

echo "📦 Updating system packages..."
apt update && apt upgrade -y

echo "🛠️ Installing build tools..."
apt install -y \
  build-essential \
  cmake \
  git \
  pkg-config

echo "🎙️ Installing audio dependencies..."
apt install -y \
  alsa-utils \
  libasound2-dev \
  portaudio19-dev

echo "📷 Installing vision dependencies..."
apt install -y \
  libopencv-dev \
  v4l-utils

echo "🧠 Installing optional ML/audio libs..."
apt install -y \
  libspeex-dev \
  libspeexdsp-dev

echo "✅ Dependency setup complete."
echo "You may now build the Pi 3B application."
