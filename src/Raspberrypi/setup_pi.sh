#!/usr/bin/env bash
#
# Fresh-Pi setup for the WRO Future Engineers robot code.
#
# Target: Raspberry Pi 5, Raspberry Pi OS Bookworm/Trixie (64-bit).
# Run it once on a new Pi, from inside the cloned repo:
#
#     git clone https://github.com/chankrrn/WRO-FE-YBR.git
#     cd WRO-FE-YBR/src/Raspberrypi
#     bash setup_pi.sh
#
# Then reboot (the script says so at the end) and:
#
#     uv run python main.py qualification --dry-run
#
set -euo pipefail

echo "==> 1/6  System packages"
sudo apt update
sudo apt full-upgrade -y
sudo apt install -y \
    git curl \
    python3-venv python3-pip \
    python3-picamera2 python3-libcamera \
    libglib2.0-0t64 libgl1 \
    i2c-tools v4l-utils

# picamera2 lives in the system Python (apt only, no working pip wheel), so the
# project venv is created with --system-site-packages further down to see it.

echo "==> 2/6  Enable I2C (BNO055 compass + DFRobot expansion board)"
sudo raspi-config nonint do_i2c 0
# Camera is on by default (camera_auto_detect=1); this is a no-op if already set.
sudo raspi-config nonint do_camera 0 2>/dev/null || true

echo "==> 3/6  Device permissions"
# dialout: /dev/ttyACM0 (Arduino UNO R4) and /dev/ttyUSB0 (RPLidar C1)
# i2c/gpio/video: expansion board, GPIO, camera
sudo usermod -aG dialout,i2c,gpio,spi,video,render "$USER"

echo "==> 4/6  uv (Python project manager)"
if ! command -v uv >/dev/null 2>&1; then
    curl -LsSf https://astral.sh/uv/install.sh | sh
    export PATH="$HOME/.local/bin:$PATH"
fi
uv --version

echo "==> 5/6  Python dependencies"
cd "$(dirname "$0")"
# --system-site-packages is what lets the venv import the apt-installed
# picamera2. Without it camera_manager.py silently falls back to the stub in
# utils/fake_picamera2.py and every frame comes back empty.
uv venv --system-site-packages
uv sync --frozen

echo "==> 6/6  Runtime directories"
# camera_manager.py writes debug recordings to src/videos/ (gitignored).
mkdir -p ../videos

echo
echo "=============================================="
echo " Done. REBOOT NOW - the group changes and the"
echo " I2C overlay only take effect after a reboot:"
echo
echo "     sudo reboot"
echo
echo " After the reboot, check the install with:"
echo "     cd $(pwd)"
echo "     i2cdetect -y 1              # expansion board + BNO055 should appear"
echo "     ls /dev/ttyACM0 /dev/ttyUSB0  # Arduino + lidar"
echo "     uv run python main.py qualification --dry-run"
echo "=============================================="
