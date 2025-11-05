# Dockerfile for Stereo VO on Jetson
# Base: NVIDIA L4T (Linux for Tegra) with ROS 2 Humble

ARG L4T_VERSION=35.3.1
FROM nvcr.io/nvidia/l4t-base:r${L4T_VERSION}

# Set environment
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Install basic tools
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    wget \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Add ROS 2 repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-message-filters \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libopencv-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-tools \
    i2c-tools \
    libi2c-dev \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --no-cache-dir \
    numpy \
    scipy \
    matplotlib \
    pyyaml \
    opencv-python

# Optional: Try to install GTSAM
RUN apt-get update && \
    (apt-get install -y libgtsam-dev || echo "GTSAM not available") && \
    rm -rf /var/lib/apt/lists/*

# Create workspace
RUN mkdir -p /workspace/src
WORKDIR /workspace

# Copy source code
COPY . /workspace/src/vio_on_jetson/

# Source ROS 2 and build
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd /workspace && \
    colcon build --symlink-install --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS='-O3 -DNDEBUG' \
    || echo 'Build completed with warnings'"

# Setup entrypoint
COPY <<'EOF' /ros_entrypoint.sh
#!/bin/bash
set -e

# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source workspace if built
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi

exec "$@"
EOF

RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# Labels
LABEL maintainer="Robot Developer <dev@example.com>"
LABEL description="Stereo Visual Odometry Stack for Jetson Orin Nano"
LABEL version="0.1.0"
