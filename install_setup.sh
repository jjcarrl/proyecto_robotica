#!/bin/bash
# Full environment setup for Raspberry Pi 5 + ROS2 Jazzy + Camera Module 3
# Ubuntu 24.04 (Noble) - run with: bash install_setup.sh

set -e

WORKSPACE_DIR="$(cd "$(dirname "$0")" && pwd)"
BASHRC="$HOME/.bashrc"
ROS_DISTRO="jazzy"

echo "======================================"
echo " Setup: ROS2 Jazzy + Camera + Deps"
echo " Workspace: $WORKSPACE_DIR"
echo "======================================"

# ── 1. System locale ────────────────────────────────────────────────────────
echo "[1/8] Ensuring UTF-8 locale..."
sudo apt-get install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ── 2. ROS2 Jazzy ────────────────────────────────────────────────────────────
echo "[2/8] Installing ROS2 Jazzy..."
sudo apt-get install -y software-properties-common curl
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt-get update
sudo apt-get install -y ros-jazzy-ros-base ros-dev-tools

# Extra ROS2 packages needed by the packages in this workspace
sudo apt-get install -y \
    ros-jazzy-cv-bridge \
    ros-jazzy-tf2-ros \
    ros-jazzy-nav-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-joy \
    ros-jazzy-std-msgs

# ── 3. colcon ────────────────────────────────────────────────────────────────
echo "[3/8] Installing colcon..."
sudo apt-get install -y python3-colcon-common-extensions

# ── 4. Camera Module 3 (libcamera + picamera2) ───────────────────────────────
echo "[4/8] Installing Camera Module 3 support..."
sudo apt-get install -y \
    libcamera-apps \
    libcamera-dev \
    v4l-utils \
    python3-picamera2 \
    python3-libcamera

# Enable v4l2-compat so OpenCV can also reach the camera if needed
sudo apt-get install -y libcamera-v4l2 2>/dev/null || true

# Make sure the camera overlay is set (camera_auto_detect=1 should already be there)
if ! grep -q "camera_auto_detect" /boot/firmware/config.txt; then
    echo "camera_auto_detect=1" | sudo tee -a /boot/firmware/config.txt
fi

# Add user to video group
sudo usermod -aG video "$USER" 2>/dev/null || true

# ── 5. OpenCV ─────────────────────────────────────────────────────────────────
echo "[5/8] Installing OpenCV..."
sudo apt-get install -y \
    python3-opencv \
    libopencv-dev

# ── 6. Python dependencies ───────────────────────────────────────────────────
echo "[6/8] Installing Python dependencies..."
# gpiozero + lgpio (Pi 5 uses lgpio backend)
sudo apt-get install -y python3-gpiozero python3-lgpio lgpio

# numpy (already likely present but just in case)
sudo apt-get install -y python3-numpy

# pip packages not available in apt
pip3 install --break-system-packages mpu6050-raspberrypi simple-pid flask 2>/dev/null || \
pip3 install mpu6050-raspberrypi simple-pid flask

# ── 7. I2C (for MPU6050) ─────────────────────────────────────────────────────
echo "[7/8] Enabling I2C for MPU6050..."
sudo apt-get install -y python3-smbus i2c-tools
# Enable i2c-dev module
if ! grep -q "^i2c-dev" /etc/modules; then
    echo "i2c-dev" | sudo tee -a /etc/modules
fi
sudo modprobe i2c-dev 2>/dev/null || true
sudo usermod -aG i2c "$USER" 2>/dev/null || true

# ── 8. Autosource ROS2 + workspace ───────────────────────────────────────────
echo "[8/8] Setting up autosourcing in ~/.bashrc..."

ROS_SOURCE="source /opt/ros/${ROS_DISTRO}/setup.bash"
WS_SOURCE="source ${WORKSPACE_DIR}/install/setup.bash"
COLCON_CD="source /usr/share/colcon_cd/function/colcon_cd.sh"
COLCON_ARGCOMPLETE="source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash"

add_if_missing() {
    local line="$1"
    if ! grep -qF "$line" "$BASHRC"; then
        echo "$line" >> "$BASHRC"
    fi
}

add_if_missing ""
add_if_missing "# ROS2 Jazzy"
add_if_missing "$ROS_SOURCE"
add_if_missing "$COLCON_CD"
add_if_missing "$COLCON_ARGCOMPLETE"
add_if_missing "# Workspace overlay (sourced only if built)"
add_if_missing "[ -f ${WORKSPACE_DIR}/install/setup.bash ] && $WS_SOURCE"
add_if_missing "export ROS_DOMAIN_ID=0"

# Source ROS for this shell session so the build below works
source /opt/ros/${ROS_DISTRO}/setup.bash

# ── Build the workspace ───────────────────────────────────────────────────────
echo ""
echo "======================================"
echo " Building workspace with colcon..."
echo "======================================"
cd "$WORKSPACE_DIR"
colcon build --symlink-install

echo ""
echo "======================================"
echo " DONE!"
echo "======================================"
echo ""
echo "Next steps:"
echo "  1. Reboot (or run: source ~/.bashrc) to apply all changes"
echo "  2. Test camera:  python3 -c \"from picamera2 import Picamera2; cam=Picamera2(); cam.start(); print('Camera OK')\""
echo "  3. Check I2C:    sudo i2cdetect -y 1   (MPU6050 should appear at 0x68)"
echo "  4. Run nodes:    ros2 run sensors camera_node"
echo ""
