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
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

## Quick Start

### Test Individual Components

```bash
# Test Point-LIO with L2
ros2 launch point_lio mapping_unilidar_l2.launch.py

# Test GNSS
ros2 launch tank_sensors gnss.launch.py

# Test ODrive (after hardware connection)
ros2 launch tank_control odrive_interface.launch.py
```

### Launch Full System

```bash
# Complete navigation stack
ros2 launch tank_bringup tank_full.launch.py

# Or step-by-step:
# Terminal 1: Hardware
ros2 launch tank_sensors hardware.launch.py

# Terminal 2: Localization
ros2 launch tank_localization localization.launch.py

# Terminal 3: Perception
ros2 launch tank_perception perception.launch.py

# Terminal 4: Navigation
ros2 launch tank_navigation navigation.launch.py
```

## Development Status

- [x] Workspace structure created
- [x] Custom messages defined
- [ ] Sensor integration (Phase 1)
- [ ] Point-LIO + GNSS fusion (Phase 1)
- [ ] ODrive control (Phase 1)
- [ ] Ground extraction (Phase 2)
- [ ] DWA planner (Phase 2)
- [ ] Camera segmentation (Phase 3)
- [ ] Reverse mode (Phase 4)
- [ ] Field testing (Phase 5)

## Documentation

- **Planning docs:** `../` (tank_plan.txt, workspace_structure.md)
- **Field reference:** `../QUICK_REFERENCE.md`
- **Project status:** `../PROJECT_STATUS.md`

## Contributing

See implementation phases in `../workspace_structure.md`.

**Current Phase:** Phase 0 - Workspace Setup ✅
**Next Phase:** Phase 1 - Core Localization & Control

