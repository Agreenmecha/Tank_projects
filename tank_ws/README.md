# Tank ROS2 Workspace

ROS2 Humble workspace for autonomous tracked tank navigation system.

## Package Structure

```
tank_ws/src/
├── tank_bringup/          # Launch files & top-level configs
├── tank_control/          # ODrive motor control + safety
├── tank_description/      # URDF & robot model
├── tank_localization/     # Point-LIO + GNSS fusion
├── tank_msgs/             # Custom messages & services
├── tank_navigation/       # DWA planner + reverse mode
├── tank_odrive_can/       # Optional: CAN bus motor control
├── tank_perception/       # LiDAR + camera fusion
├── tank_sensors/          # Sensor driver wrappers
├── tank_utils/            # Tools, logging, monitoring
└── external/              # External dependencies (git cloned)
    ├── point_lio_ros2/
    ├── unilidar_sdk2/
    ├── ublox_dgnss/
    ├── isaac_ros_common/
    ├── isaac_ros_dnn_inference/
    ├── isaac_ros_image_pipeline/
    ├── patchwork-plusplus/ (Phase 2)
    └── odrive_ros2_control/ (optional USB)
```

## Setup

### 1. Clone External Dependencies

```bash
cd src/external

# Point-LIO (LiDAR-inertial odometry)
git clone https://github.com/dfloreaa/point_lio_ros2.git

# Unitree L2 LiDAR driver
git clone https://github.com/unitreerobotics/unilidar_sdk2.git

# GNSS driver
git clone https://github.com/aussierobots/ublox_dgnss.git

# Isaac ROS (Camera segmentation with TensorRT)
git clone --branch release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone --branch release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_inference.git
git clone --branch release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git

# (Optional - Phase 2)
# Ground extraction
# git clone https://github.com/url-kaist/patchwork-plusplus.git

# (Optional - if using USB for ODrive instead of CAN)
# git clone -b humble-fw-v0.5.3 https://github.com/Factor-Robotics/odrive_ros2_control.git
```

### 2. Install Dependencies

```bash
cd ~/Tank_projects/tank_ws

# Install ROS2 dependencies
rosdep install --from-paths src --ignore-src -r -y

# Install Isaac ROS dependencies (for JetPack 6.2.1)
sudo apt install -y \
    libvpi3 vpi3-dev \
    nvidia-tensorrt \
    python3-libnvinfer \
    python3-libnvinfer-dev \
    ros-humble-gscam

# Install additional tools
sudo apt install -y \
    ros-humble-nav2-bringup \
    ros-humble-robot-localization \
    libeigen3-dev \
    libpcl-dev
```

### 3. Build Workspace

```bash
cd ~/Tank_projects/tank_ws

# Source ROS2
source /opt/ros/humble/setup.bash

# Build (first time - may take 15-20 minutes)
# Build only specific packages first to check for errors:
colcon build --packages-select tank_msgs --symlink-install

# Build core tank packages:
colcon build --packages-select tank_sensors tank_localization tank_bringup --symlink-install

# Build external packages:
colcon build --packages-up-to \
  unitree_lidar_ros2 \
  point_lio \
  ublox_dgnss \
  isaac_ros_dnn_image_encoder \
  isaac_ros_tensor_rt \
  isaac_ros_image_proc \
  --symlink-install

# Or build everything at once:
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

**Note:** External packages are git cloned (not submodules) for easier development.

## Quick Start

### Phase 1: Test Sensors & Localization

**Important:** These commands assume you're on the Jetson with hardware connected.

#### 1. Test GNSS Only
```bash
# Make sure ZED-F9P is connected (check: ls /dev/ttyACM*)
ros2 launch tank_sensors gnss.launch.py

# In another terminal, check output:
ros2 topic echo /gnss/fix --no-arr
ros2 topic echo /gnss/dop
```

#### 2. Test Single L2 LiDAR
```bash
# Test front L2 (ensure IP is 192.168.123.123)
cd ~/Tank_projects/tank_ws/src/external/unilidar_sdk2/unitree_lidar_ros2
source ~/Tank_projects/tank_ws/install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py

# Check output:
ros2 topic echo /unilidar/cloud --no-arr
ros2 topic echo /unilidar/imu
```

#### 3. Test Dual L2 Setup
```bash
# Both L2s must have different IPs (123 and 124)
ros2 launch tank_sensors lidar_dual.launch.py

# Check topics:
ros2 topic list | grep lidar
# Should see:
#   /lidar_front/pointcloud
#   /lidar_front/imu
#   /lidar_rear/pointcloud
#   /lidar_rear/imu
```

#### 4. Test Point-LIO Localization
```bash
# Launches Point-LIO with front L2
ros2 launch tank_localization point_lio.launch.py

# Check odometry output:
ros2 topic echo /point_lio/odom
# Move the tank and verify pose increments!
```

#### 5. Launch Everything Together
```bash
# Full sensor suite + Point-LIO
ros2 launch tank_bringup sensors_localization.launch.py

# With RViz for visualization:
ros2 launch tank_bringup sensors_localization.launch.py rviz:=true
```

### Phase 2+: Full Navigation (TODO)

```bash
# Complete navigation stack (not implemented yet)
ros2 launch tank_bringup tank_full.launch.py
```

## Development Status

### Phase 0: Setup ✅
- [x] Workspace structure created
- [x] Custom messages defined (5 msgs, 2 srvs)
- [x] 10 ROS2 packages created

### Phase 1: Sensors & Localization (IN PROGRESS)
- [x] External packages cloned (Point-LIO, unilidar_sdk2, ublox_dgnss, Isaac ROS)
- [x] GNSS config & launch files (ZED-F9P)
- [x] Dual L2 LiDAR launch files
- [x] Camera integration (e-CAM25 with gscam + nvarguscamerasrc)
- [x] Point-LIO configuration for L2
- [x] Master sensors_localization launch file
- [x] Isaac ROS packages cloned (common, dnn_inference, image_pipeline)
- [ ] Test on Jetson hardware (pending JetPack 6.2.1 + camera drivers)
- [ ] ODrive CAN control
- [ ] Encoder odometry
- [ ] Safety monitoring node

### Phase 2: Perception & Navigation (TODO)
- [ ] Ground extraction (Patchwork++)
- [ ] Isaac ROS TensorRT model preparation (SegFormer-B0)
- [ ] Camera segmentation inference node
- [ ] LiDAR+Camera costmap fusion
- [ ] DWA planner (tracked vehicle kinematics)
- [ ] Basic waypoint following

### Phase 3: Advanced Behavior (TODO)
- [ ] Reverse mode
- [ ] Recovery behaviors (stuck, tip detection)
- [ ] Pitch-based speed control
- [ ] 30° slope validation

### Phase 4: Field Testing & Iteration (TODO)
- [ ] T1/T2/T3 test scenarios
- [ ] KPI collection
- [ ] Nvblox 3D reconstruction (LiDAR+Camera)
- [ ] Isaac Sim integration for offline testing

## Documentation

- **Planning docs:** `../` (tank_plan.txt, workspace_structure.md)
- **Field reference:** `../QUICK_REFERENCE.md`
- **Project status:** `../PROJECT_STATUS.md`

## Contributing

See implementation phases in `../workspace_structure.md`.

**Current Phase:** Phase 0 - Workspace Setup ✅
**Next Phase:** Phase 1 - Core Localization & Control

