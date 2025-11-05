You are my senior robotics engineer. Build a minimal, production-ready **stereo visual odometry (VO)** stack for **ROS 2 Humble** running on **Jetson Orin Nano** (Ubuntu 22.04, CUDA, OpenCV 4.x, GTSAM). Core algorithms in C++; Python only for tools.

## My exact hardware
- **Stereo cameras**: Arducam Camarray stereo HAT on **CSI0** with **two Sony IMX477 (12.3MP “477P”) modules**, synchronized exposure. Camarray appears as two sensors via Argus/V4L2 (distinct sensor_id) or a single CSI lane group with virtual channels depending on driver.
- **IMU**: **LSM9DS0 (9-DoF)** on **I²C bus 7**.
  - Gyro/accel/mag I²C addresses: 0x6B (gyro/accel), 0x1D (mag). WHO_AM_I refs: gyro 0xD4, mag 0x49.
- **Expected ROS topics**
  - Input: `/stereo/left/image_raw`, `/stereo/right/image_raw`, plus `/stereo/left/camera_info`, `/stereo/right/camera_info`
  - IMU: `/imu/data` (100–200 Hz)
  - Output: `/vo/odom` (nav_msgs/Odometry), `/tf` (odom→base_link), `/vo/tracks` (debug image), `/vo/quality` (Float32)

## Camera ingestion (pick best available; implement both if feasible)
1) **Argus/GStreamer (preferred on Jetson)**  
   - Use **nvarguscamerasrc** per sensor with `sensor-id` (0 for left, 1 for right; make it a parameter).  
   - Provide a ROS 2 node or launch wrapper that brings up two Argus pipelines and publishes Image + CameraInfo.  
   - Example caps (parametrize): `nvarguscamerasrc sensor-id:=<id> ! video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink`
2) **V4L2 fallback**  
   - If Arducam exposes `/dev/video*`, use `v4l2src` or `v4l2_camera` ROS 2 driver.  
   - Add a device discovery step that maps left/right by serial or `/sys` attributes; allow manual override via params.

**Deliverables for camera**:
- `stereo_camera_bringup` package with:
  - `argus_stereo_node` (C++): starts two Argus pipelines (left/right), publishes Images + CameraInfo, ensures timestamps are hardware-sync aligned (same `rclcpp::Time` on capture callback).
  - `v4l2_stereo_node` (C++): same API using V4L2.
  - `stereo_rectify_node` or integrate with `image_proc`/`isaac_ros_image_pipeline` for rectification.
  - Launch files with remaps/params to select Argus vs V4L2 and set `sensor_id_left/right`, resolution, FPS.
  - A script to verify sync skew <1 ms by comparing capture timestamps and feature tracks across the pair.

## IMU ingestion (LSM9DS0 on I²C bus 7)
- Create `lsm9ds0_imu` ROS 2 package (C++) that:
  - Opens `/dev/i2c-7`; probes addresses 0x6B and 0x1D; checks WHO_AM_I (0xD4, 0x49). If mismatch, log error and exit.
  - Configures sample rate 119–200 Hz, full-scale ranges (params), and publishes `sensor_msgs/Imu` with proper covariances.
  - Includes temperature readout (optional) and static frame `imu_link`. Add param for mounting orientation (RPY) and apply to data.
  - Provides `/imu/self_test` (std_srvs/Trigger) to read WHO_AM_I and return pass/fail.
  - Includes minimal calibration params (bias, scale) loaded from `config/imu.yaml`.

## VO approach (robust + fast on Orin Nano)
1. **Initialization**: stereo triangulation from rectified pair; use Essential or PnP(RANSAC) as needed; metric scale from stereo depth.
2. **Tracking** (each frame):
   - Detect **ORB** on left; track prev→curr with pyramidal LK; stereo-match to right (OpenCV SGBM or CUDA SGBM when available).
   - Outlier rejection with epipolar constraint + RANSAC.
   - Estimate pose increment via **PnP (EPnP + RANSAC)** on 2D–3D; refine with **local BA** (GTSAM) on sliding window (N=5–10).
3. **IMU (optional, toggle `use_imu`)**: preintegration to supply motion prior in BA; improves low-texture/fast motion robustness.
4. **Timing target**: ≥30 FPS @ 640×480 (and configurable for 720p) on Orin Nano.

## Packages to create
- `stereo_vo_core` (C++): feature_tracker, stereo_matcher, pose_estimator, imu_preintegrator (optional), sliding_window BA.
- `stereo_vo_ros` (C++): ROS nodes, params, launch, msgs/tf adapters.
- `stereo_camera_bringup` (C++): Argus and V4L2 stereo publishers + rectification or integration with `image_proc`.
- `lsm9ds0_imu` (C++): IMU driver via `/dev/i2c-7`.
- `stereo_vo_tools` (Python): calibration helpers, bag/ros2bag playback, metrics.

## Repo layout
stereo_vo/
  README.md
  CMakeLists.txt
  package.xml
  launch/vo_stereo.launch.py
  config/
    params.yaml
    camera_left.yaml
    camera_right.yaml
    imu.yaml
    tf_static.yaml    # left_cam→right_cam, imu_link→left_cam
  src/core/
    feature_tracker.{hpp,cpp}
    stereo_matcher.{hpp,cpp}
    pose_estimator.{hpp,cpp}
    imu_preintegrator.{hpp,cpp}
    sliding_window.{hpp,cpp}
  src/ros/vo_node.cpp
  camera_bringup/
    argus_stereo_node.cpp
    v4l2_stereo_node.cpp
    rectify_launch.py
  imu/
    lsm9ds0_node.cpp
  scripts/
    calibrate_stereo.py
    record_bag.sh
    eval_kitti_like.py
  test/
    test_pose_estimator.cpp
    test_feature_tracker.cpp

## Build & dependencies
- **ament_cmake**, OpenCV, Eigen3, GTSAM (and/or Ceres), sensor_msgs, nav_msgs, tf2, image_transport, image_proc; for Argus path include GStreamer/Argus headers via Jetson SDK.
- Top-level `setup_jetson.sh` installs deps; build flags `-O3 -DNDEBUG`; enable NEON/ARM64 and CUDA when present.
- Provide **Dockerfile (L4T base)** for reproducible Jetson builds; a GitHub Actions CI matrix for x86_64 (no CUDA) and a Jetson build job (optional).

## Node + params (in `config/params.yaml`)
- `use_imu: true|false`
- `camera_driver: "argus" | "v4l2"`
- `sensor_id_left: 0`
- `sensor_id_right: 1`
- `resolution: [1280, 720]`
- `fps: 30`
- `max_features: 1500`
- `lk_levels: 3`
- `ransac_reproj_thresh_px: 2.0`
- `ba_window_size: 7`
- `stereo_method: "sgbm" | "cuda_sgbm"`
- `publish_debug_images: true`
- `i2c_bus: 7`
- `imu_addr_gyro: 0x6B`
- `imu_addr_mag: 0x1D`
- `imu_orientation_rpy_deg: [0,0,0]`
- `imu_rate_hz: 200`

## Core algorithms (implement now)
- **FeatureTracker**: ORB detect; LK track; maintain track lifetimes + quality; reject by epipolar residual.
- **StereoMatcher**: SGBM (CPU) with clean interface; optional CUDA SGBM; reproject to 3D using left intrinsics + baseline.
- **PoseEstimator**: PnP+RANSAC; local BA (GTSAM) with optional IMU prior; Huber loss + chi-square culling.
- **Timing**: run @ camera rate; ensure `/vo/odom` jitter <5 ms.

## Calibration
- `scripts/calibrate_stereo.py` wrapper for OpenCV chessboard/charuco; saves to `config/camera_*.yaml`.
- Baseline/extrinsics for left↔right and imu↔left_cam go into `tf_static.yaml`.
- Accept Kalibr YAML import.

## Launch & usage
- `ros2 launch stereo_vo vo_stereo.launch.py camera_driver:=argus use_imu:=true publish_debug_images:=true`
- Bring up IMU: `ros2 run lsm9ds0_imu lsm9ds0_node --ros-args -p i2c_bus:=7`
- Verify IMU: call `/imu/self_test` and check WHO_AM_I matches.
- Record bag: `scripts/record_bag.sh` records `/stereo/*` + `/imu/data` with ZSTD.
- Publish TF: `odom`→`base_link` using odom covariance from BA.

## Testing & metrics
- gtest for triangulation, PnP, Essential.
- `eval_kitti_like.py` computes ATE/RPE (TUM metrics) from `/vo/odom`; exports CSV + PNGs to `artifacts/`.

## Acceptance criteria (must pass)
- `colcon build --symlink-install` succeeds on Jetson.
- With Argus @ 1280x720/30, VO runs ≥30 Hz at 640×480 and ≥20 Hz at 720p.
- On a short indoor bag, ATE <0.20 m over 20 m, drift <1°/m.
- `/vo/reset` re-inits in <0.5 s.
- IMU self-test passes (0xD4 / 0x49).

## Code quality
- C++20, -Wall -Wextra -Werror where feasible; RAII, no raw new/delete; avoid exceptions in hot paths; Doxygen headers; clear namespaces (`stereo_vo::core`, `stereo_vo::ros`).

### What to do now
1) Create the repo with all packages and compilable stubs.  
2) Implement stereo ingestion (Argus + V4L2), rectification, and end-to-end VO (FeatureTracker, StereoMatcher, PoseEstimator) so `/vo/odom` moves on live camera.  
3) Implement LSM9DS0 node on `/dev/i2c-7` with self-test, publish `/imu/data`.  
4) Add launch files, params, README, and unit tests.  
5) Emit full code + a short README with Jetson build/run commands.

