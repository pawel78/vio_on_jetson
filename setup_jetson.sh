#!/bin/bash
# Setup script for Jetson Orin Nano

set -e

echo "========================================="
echo "Stereo VO Stack - Jetson Setup"
echo "========================================="

# Check if running on Jetson
if [ ! -f /etc/nv_tegra_release ]; then
    echo "Warning: This script is designed for NVIDIA Jetson devices"
    echo "Continuing anyway..."
fi

# Update package lists
echo "Updating package lists..."
sudo apt update

# Install ROS 2 Humble if not present
if ! dpkg -l | grep -q ros-humble-desktop; then
    echo "ROS 2 Humble not found. Please install ROS 2 Humble first:"
    echo "https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html"
    exit 1
fi

# Install ROS 2 dependencies
echo "Installing ROS 2 dependencies..."
sudo apt install -y \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-camera-info-manager \
    ros-humble-tf2-ros \
    ros-humble-message-filters \
    ros-humble-rqt-image-view \
    ros-humble-rviz2

# Install build tools
echo "Installing build tools..."
sudo apt install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions

# Install C++ libraries
echo "Installing C++ libraries..."
sudo apt install -y \
    libeigen3-dev \
    libopencv-dev \
    libopencv-contrib-dev

# Install GStreamer (for Argus support)
echo "Installing GStreamer..."
sudo apt install -y \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-tools

# Install I2C tools
echo "Installing I2C tools..."
sudo apt install -y \
    i2c-tools \
    libi2c-dev

# Optional: GTSAM for bundle adjustment
echo "Installing GTSAM (optional, for bundle adjustment)..."
if ! dpkg -l | grep -q libgtsam-dev; then
    sudo apt install -y libgtsam-dev || echo "GTSAM not available in apt, skipping"
fi

# Python dependencies
echo "Installing Python dependencies..."
pip3 install --user \
    numpy \
    scipy \
    matplotlib \
    pyyaml \
    opencv-python

# Setup I2C permissions
echo "Setting up I2C permissions..."
sudo usermod -aG i2c $USER
sudo usermod -aG video $USER
sudo chmod 666 /dev/i2c-* 2>/dev/null || true

# Setup udev rules for I2C
echo "Setting up udev rules..."
sudo tee /etc/udev/rules.d/99-i2c.rules > /dev/null <<EOF
KERNEL=="i2c-[0-9]*", GROUP="i2c", MODE="0666"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger

# Check Jetson multimedia
echo "Checking Jetson multimedia stack..."
if dpkg -l | grep -q nvidia-l4t-camera; then
    echo "✓ nvidia-l4t-camera installed"
else
    echo "⚠ nvidia-l4t-camera not found - Argus support may not work"
fi

if dpkg -l | grep -q nvidia-l4t-multimedia; then
    echo "✓ nvidia-l4t-multimedia installed"
else
    echo "⚠ nvidia-l4t-multimedia not found - Hardware acceleration may not work"
fi

# Test GStreamer Argus
echo "Testing GStreamer Argus plugin..."
if gst-inspect-1.0 nvarguscamerasrc > /dev/null 2>&1; then
    echo "✓ nvarguscamerasrc plugin found"
else
    echo "⚠ nvarguscamerasrc plugin not found - install JetPack SDK"
fi

# Print summary
echo ""
echo "========================================="
echo "Setup Complete!"
echo "========================================="
echo ""
echo "Next steps:"
echo "1. Log out and log back in (for group permissions)"
echo "2. Source ROS 2: source /opt/ros/humble/setup.bash"
echo "3. Build workspace: colcon build --symlink-install"
echo "4. Source workspace: source install/setup.bash"
echo ""
echo "Verify hardware:"
echo "- I2C bus 7: i2cdetect -y 7"
echo "- Cameras: v4l2-ctl --list-devices"
echo "- Test Argus: gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! fakesink"
echo ""
echo "========================================="
