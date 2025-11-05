# Stereo Visual Odometry Stack for Jetson Orin Nano

Production-ready stereo visual odometry (VO) implementation for ROS 2 Humble on Jetson Orin Nano with Arducam Camarray stereo HAT (Sony IMX477) and LSM9DS0 IMU.

## Features

- **Stereo Visual Odometry**: ORB feature detection, pyramidal LK tracking, SGBM stereo matching, PnP pose estimation with RANSAC
- **Bundle Adjustment**: Sliding window BA with GTSAM (optional)
- **IMU Integration**: LSM9DS0 9-DOF IMU driver with preintegration (optional)
- **Dual Camera Support**: 
  - Argus/GStreamer (nvarguscamerasrc) for Jetson native acceleration
  - V4L2 fallback for standard USB/CSI cameras
- **Real-time Performance**: ≥30 Hz @ 640×480, ≥20 Hz @ 720p on Jetson Orin Nano
- **C++20**: Modern C++, RAII, no raw pointers, comprehensive error handling

## Hardware Requirements

- **Platform**: NVIDIA Jetson Orin Nano (Ubuntu 22.04, JetPack 5.x+)
- **Cameras**: Arducam Camarray stereo HAT with 2× Sony IMX477 (12.3MP) on CSI0
- **IMU**: LSM9DS0 (9-DOF) on I²C bus 7
  - Gyro/Accel address: 0x6B (WHO_AM_I: 0xD4)
  - Magnetometer address: 0x1D (WHO_AM_I: 0x49)

## Packages

| Package | Description |
|---------|-------------|
| `stereo_vo` | Core VO algorithms and ROS 2 node |
| `stereo_camera_bringup` | Argus and V4L2 stereo camera drivers |
| `lsm9ds0_imu` | LSM9DS0 IMU driver with I²C interface |
| `stereo_vo_tools` | Python utilities for calibration and evaluation |

## Dependencies

### System Dependencies
```bash
# Install ROS 2 Humble first
sudo apt update
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-camera-info-manager \
    ros-humble-tf2-ros \
    ros-humble-message-filters

# Build tools and libraries
sudo apt install -y \
    build-essential cmake git \
    libeigen3-dev \
    libopencv-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    i2c-tools libi2c-dev

# Optional: GTSAM for bundle adjustment
sudo apt install -y libgtsam-dev
```

### Jetson-Specific (for Argus support)
Ensure JetPack SDK is installed with:
- `nvidia-l4t-camera` (Argus camera API)
- `nvidia-l4t-multimedia` (GStreamer plugins)

## Build Instructions

```bash
# Clone repository
cd ~/ros2_ws/src
git clone <repository-url> vio_on_jetson
cd ~/ros2_ws

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build with optimizations
colcon build --symlink-install --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-O3 -DNDEBUG"

# Source workspace
source install/setup.bash
```

## Quick Start

### 1. Camera Calibration

Calibrate stereo cameras before first use:

```bash
# Run calibration (OpenCV chessboard/charuco)
ros2 run stereo_vo_tools calibrate_stereo.py \
    --left /stereo/left/image_raw \
    --right /stereo/right/image_raw \
    --pattern chessboard \
    --size 9x6 \
    --square 0.025

# Calibration files saved to stereo_vo/config/camera_left.yaml and camera_right.yaml
```

### 2. Verify IMU

Test IMU connection and WHO_AM_I:

```bash
# Check I2C devices
i2cdetect -y 7

# Start IMU node
ros2 run lsm9ds0_imu lsm9ds0_node --ros-args -p i2c_bus:=7

# Verify (should return "WHO_AM_I verification passed")
ros2 service call /imu/self_test std_srvs/srv/Trigger
```

### 3. Verify Camera Sync

Check stereo pair synchronization:

```bash
# Start cameras (Argus)
ros2 launch stereo_camera_bringup stereo_camera.launch.py driver:=argus

# In another terminal, verify sync
ros2 run stereo_camera_bringup verify_sync.py

# Expected: skew < 1 ms
```

### 4. Run Visual Odometry

Full VO stack with IMU:

```bash
# Launch complete stack
ros2 launch stereo_vo vo_stereo.launch.py \
    camera_driver:=argus \
    use_imu:=true \
    publish_debug_images:=true

# Monitor odometry
ros2 topic echo /vo/odom

# Visualize in RViz
rviz2
```

## Configuration

Main parameters in `stereo_vo/config/params.yaml`:

```yaml
use_imu: true                      # Enable IMU integration
camera_driver: "argus"             # "argus" or "v4l2"
max_features: 1500                 # Max features to track
ransac_reproj_thresh_px: 2.0       # RANSAC threshold
ba_window_size: 7                  # Sliding window size
stereo_method: "sgbm"              # "sgbm" or "cuda_sgbm"
publish_debug_images: true         # Publish /vo/tracks
```

## ROS 2 Topics

### Inputs
- `/stereo/left/image_raw` (sensor_msgs/Image): Left camera image
- `/stereo/right/image_raw` (sensor_msgs/Image): Right camera image
- `/stereo/left/camera_info` (sensor_msgs/CameraInfo): Left camera info
- `/stereo/right/camera_info` (sensor_msgs/CameraInfo): Right camera info
- `/imu/data` (sensor_msgs/Imu): IMU measurements @ 200 Hz

### Outputs
- `/vo/odom` (nav_msgs/Odometry): Visual odometry pose
- `/tf` (odom → base_link): Transform broadcast
- `/vo/tracks` (sensor_msgs/Image): Debug visualization (if enabled)
- `/vo/quality` (std_msgs/Float32): Tracking quality (inlier ratio)

### Services
- `/vo/reset` (std_srvs/Trigger): Reset VO state
- `/imu/self_test` (std_srvs/Trigger): IMU WHO_AM_I verification

## Performance

Target performance on Jetson Orin Nano:

| Resolution | Target FPS | Expected |
|------------|------------|----------|
| 640×480    | ≥30        | ~35-40   |
| 1280×720   | ≥20        | ~25-30   |

## Troubleshooting

See full README in repository for detailed troubleshooting steps for camera, IMU, and VO issues.

## License

MIT License
