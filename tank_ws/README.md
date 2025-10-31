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
└── external/              # Git submodules (dependencies)
    ├── point_lio_ros2/
    ├── unilidar_sdk2/
    ├── patchwork-plusplus/
    ├── ublox_dgnss/
    └── odrive_ros2_control/
```

## Setup

### 1. Clone External Dependencies

```bash
cd src/external

# Point-LIO (LiDAR-inertial odometry)
git clone https://github.com/dfloreaa/point_lio_ros2.git

# Unitree L2 LiDAR driver
git clone https://github.com/unitreerobotics/unilidar_sdk2.git

# Ground extraction
git clone https://github.com/url-kaist/patchwork-plusplus.git

# GNSS driver
git clone https://github.com/aussierobots/ublox_dgnss.git

# ODrive control (if using USB)
git clone -b humble-fw-v0.5.3 https://github.com/Factor-Robotics/odrive_ros2_control.git
```

### 2. Install Dependencies

```bash
cd ~/Tank_projects/tank_ws

# Install ROS2 dependencies
rosdep install --from-paths src --ignore-src -r -y

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

# Build (first time - may take 10-15 minutes)
# Build only specific packages first to check for errors:
colcon build --packages-select tank_msgs --symlink-install
colcon build --packages-select tank_sensors tank_localization tank_bringup --symlink-install

# Then build external packages:
colcon build --packages-select unitree_lidar_ros2 point_lio ublox_dgnss --symlink-install

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
- [x] External packages cloned (Point-LIO, unilidar_sdk2, ublox_dgnss)
- [x] GNSS config & launch files
- [x] Dual L2 LiDAR launch files
- [x] Camera integration (v4l2_camera)
- [x] Point-LIO configuration for L2
- [x] Master sensors_localization launch file
- [ ] Test on Jetson hardware
- [ ] ODrive CAN control
- [ ] Encoder odometry
- [ ] Safety monitoring node

### Phase 2: Navigation (TODO)
- [ ] Ground extraction (Patchwork++)
- [ ] DWA planner (tracked vehicle)
- [ ] Basic waypoint following

### Phase 3: Camera Perception (TODO)
- [ ] Isaac ROS setup
- [ ] Camera segmentation training
- [ ] LiDAR+Camera fusion

### Phase 4: Advanced (TODO)
- [ ] Reverse mode
- [ ] Recovery behaviors
- [ ] 30° slope validation

### Phase 5: Field Testing (TODO)
- [ ] T1/T2/T3 test scenarios
- [ ] KPI collection

## Documentation

- **Planning docs:** `../` (tank_plan.txt, workspace_structure.md)
- **Field reference:** `../QUICK_REFERENCE.md`
- **Project status:** `../PROJECT_STATUS.md`

## Contributing

See implementation phases in `../workspace_structure.md`.

**Current Phase:** Phase 0 - Workspace Setup ✅
**Next Phase:** Phase 1 - Core Localization & Control

