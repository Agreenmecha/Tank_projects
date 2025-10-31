# Tank Autonomous Navigation - ROS2 Workspace Structure

## Overview
This document outlines the complete workspace structure for the tracked tank autonomous navigation system using ROS2 Humble.

---

## Directory Structure

```
tank_ws/                                    # ROS2 workspace root
├── src/                                    # Source packages
│   ├── tank_bringup/                      # Launch files & top-level configs
│   │   ├── launch/
│   │   │   ├── tank_full.launch.py        # Complete system launch
│   │   │   ├── localization.launch.py     # Point-LIO + GNSS fusion
│   │   │   ├── perception.launch.py       # LiDAR + camera perception
│   │   │   ├── navigation.launch.py       # Planning & control
│   │   │   ├── teleop.launch.py          # Manual control for testing
│   │   │   └── hardware.launch.py         # Sensors + ODrive
│   │   ├── config/
│   │   │   ├── tank_params.yaml          # Master parameter file
│   │   │   ├── sensors.yaml              # Sensor configurations
│   │   │   └── rviz_tank.rviz            # RViz visualization
│   │   ├── urdf/
│   │   │   └── tank.urdf.xacro           # Robot description
│   │   └── package.xml
│   │
│   ├── tank_localization/                 # State estimation
│   │   ├── launch/
│   │   │   └── point_lio.launch.py       # Point-LIO configuration
│   │   ├── config/
│   │   │   ├── point_lio_l2.yaml         # Point-LIO params for L2
│   │   │   └── gnss_fusion.yaml          # EKF fusion config
│   │   ├── src/
│   │   │   ├── gnss_fusion_node.cpp      # GNSS → map frame fusion
│   │   │   └── odom_monitor_node.cpp     # Cross-check encoders/Point-LIO/GNSS
│   │   └── package.xml
│   │
│   ├── tank_perception/                   # Sensor processing → costmap
│   │   ├── launch/
│   │   │   ├── lidar_processing.launch.py
│   │   │   └── camera_segmentation.launch.py
│   │   ├── config/
│   │   │   ├── patchwork_pp.yaml         # Ground extraction config
│   │   │   ├── costmap_fusion.yaml       # LiDAR + camera fusion
│   │   │   └── terrain_costs.yaml        # Class → cost mapping
│   │   ├── src/
│   │   │   ├── ground_extraction_node.cpp    # Patchwork++ wrapper
│   │   │   ├── camera_projection_node.cpp    # Project segmentation to ground
│   │   │   ├── costmap_fusion_node.cpp       # Fuse LiDAR + camera costs
│   │   │   └── terrain_classifier_node.cpp   # TensorRT inference wrapper
│   │   ├── models/
│   │   │   ├── segformer_b0.onnx         # Trained model
│   │   │   └── segformer_b0_fp16.engine  # TensorRT engine
│   │   └── package.xml
│   │
│   ├── tank_navigation/                   # Planning & control
│   │   ├── launch/
│   │   │   └── navigation.launch.py
│   │   ├── config/
│   │   │   ├── global_planner.yaml       # Waypoint corridors
│   │   │   ├── dwa_local_planner.yaml    # Tracked vehicle DWA
│   │   │   ├── costmap_global.yaml
│   │   │   ├── costmap_local.yaml
│   │   │   └── recovery_behaviors.yaml
│   │   ├── src/
│   │   │   ├── global_planner_node.cpp       # Waypoint → corridor
│   │   │   ├── dwa_tracked_planner.cpp       # Custom DWA for tracks
│   │   │   ├── reverse_manager_node.cpp      # Rear LiDAR reverse logic
│   │   │   ├── recovery_manager_node.cpp     # Stuck/belly strike recovery
│   │   │   └── corridor_adapter_node.cpp     # GNSS quality → corridor width
│   │   └── package.xml
│   │
│   ├── tank_control/                      # Motor interface & safety
│   │   ├── launch/
│   │   │   └── odrive_interface.launch.py
│   │   ├── config/
│   │   │   ├── odrive_params.yaml        # Motor control settings
│   │   │   └── safety_limits.yaml        # E-stop, watchdog, pitch limits
│   │   ├── src/
│   │   │   ├── odrive_interface_node.cpp     # cmd_vel → ODrive (USB or CAN)
│   │   │   ├── encoder_odom_node.cpp         # Encoders → wheel odometry
│   │   │   ├── safety_monitor_node.cpp       # Watchdog, pitch, thermal
│   │   │   ├── slip_detector_node.cpp        # Encoder vs Point-LIO
│   │   │   └── estop_handler_node.cpp        # Hardware E-stop integration
│   │   └── package.xml
│   │
│   ├── tank_odrive_can/                   # OPTION: Custom CAN interface (if USB doesn't work)
│   │   ├── include/tank_odrive_can/
│   │   │   ├── odrive_can_interface.hpp
│   │   │   └── can_socket_wrapper.hpp
│   │   ├── src/
│   │   │   ├── odrive_can_node.cpp           # ROS2 node for CAN control
│   │   │   ├── odrive_can_interface.cpp      # ODrive CAN Simple Protocol
│   │   │   └── can_socket_wrapper.cpp        # Linux SocketCAN wrapper
│   │   ├── config/
│   │   │   └── odrive_can_params.yaml        # CAN IDs, bitrate, axis mapping
│   │   ├── launch/
│   │   │   └── odrive_can.launch.py
│   │   └── package.xml
│   │
│   ├── tank_sensors/                      # Sensor drivers & wrappers
│   │   ├── launch/
│   │   │   ├── lidar_front.launch.py     # Front L2 driver
│   │   │   ├── lidar_rear.launch.py      # Rear L2 driver
│   │   │   ├── camera.launch.py          # e-CAM25_CUONX driver
│   │   │   └── gnss.launch.py            # ZED-F9P wrapper (uses ublox_dgnss)
│   │   ├── config/
│   │   │   ├── lidar_l2_front.yaml
│   │   │   ├── lidar_l2_rear.yaml
│   │   │   ├── camera_ar0234.yaml
│   │   │   └── gnss_f9p.yaml             # Device serial, output topics
│   │   └── package.xml
│   │
│   ├── tank_description/                  # URDF, meshes, transforms
│   │   ├── urdf/
│   │   │   ├── tank.urdf.xacro           # Main robot description
│   │   │   ├── sensors.urdf.xacro        # Sensor mounts & extrinsics
│   │   │   └── tracks.urdf.xacro         # Footprint & collision
│   │   ├── meshes/
│   │   │   └── tank_chassis.stl          # Visual mesh (if available)
│   │   ├── config/
│   │   │   └── extrinsics.yaml           # Calibrated sensor transforms
│   │   └── package.xml
│   │
│   ├── tank_msgs/                         # Custom message definitions
│   │   ├── msg/
│   │   │   ├── SafetyStatus.msg          # E-stop, pitch, thermal state
│   │   │   ├── TerrainCost.msg           # Fused costmap cell
│   │   │   ├── SlipStatus.msg            # Encoder vs LiDAR mismatch
│   │   │   ├── GNSSQuality.msg           # Satellite count, HDOP, covariance
│   │   │   └── RecoveryAction.msg        # Recovery behavior trigger
│   │   ├── srv/
│   │   │   ├── EnableReverse.srv         # Rear LiDAR safety gate check
│   │   │   └── TriggerRecovery.srv       # Manual recovery trigger
│   │   └── package.xml
│   │
│   ├── tank_utils/                        # Logging, monitoring, tools
│   │   ├── scripts/
│   │   │   ├── rosbag_logger.py          # Automated logging
│   │   │   ├── kpi_dashboard.py          # Real-time KPI display
│   │   │   ├── calibration_tools.py      # Extrinsics calibration helpers
│   │   │   └── model_converter.py        # ONNX → TensorRT conversion
│   │   ├── launch/
│   │   │   └── logging.launch.py
│   │   ├── config/
│   │   │   └── logging_topics.yaml       # What to record
│   │   └── package.xml
│   │
│   └── external/                          # Third-party dependencies (submodules)
│       ├── point_lio_ros2/               # git submodule: Point-LIO (includes L2 support!)
│       │   # Launch: mapping_unilidar_l2.launch.py
│       ├── unilidar_sdk2/                # git submodule: Unitree L2 driver
│       │   # Topics: /unilidar/cloud, /unilidar/imu (6-axis, hardware time-synced)
│       ├── patchwork-plusplus/           # git submodule: Ground extraction
│       ├── ublox_dgnss/                  # git submodule: ZED-F9P GNSS driver
│       └── odrive_ros2_control/          # git submodule: ODrive interface
│           # Branch: humble-fw-v0.5.3 OR custom fork for fw v0.5.6
│           # See "ODrive Integration Notes" section below
│
├── isaac_ros_ws/                          # Separate Isaac ROS workspace (Docker)
│   └── src/
│       ├── isaac_ros_common/
│       ├── isaac_ros_dnn_inference/
│       └── isaac_ros_image_pipeline/
│
├── install/                               # Build artifacts (gitignored)
├── build/                                 # Build artifacts (gitignored)
├── log/                                   # ROS logs (gitignored)
│
├── data/                                  # Datasets & models
│   ├── rosbags/                          # Recorded data
│   │   ├── T1_parking_lot/
│   │   ├── T2_moderate_terrain/
│   │   └── T3_rugged_terrain/
│   ├── camera_calibration/
│   │   ├── ar0234_intrinsics.yaml
│   │   └── lidar_camera_extrinsics.yaml
│   ├── segmentation_training/
│   │   ├── images/                       # Raw camera frames
│   │   ├── labels/                       # Annotated terrain masks
│   │   └── train_test_split.txt
│   └── maps/                             # Pre-built maps (if any)
│
├── docker/                                # Containerization
│   ├── Dockerfile.ros2                   # Base ROS2 Humble image
│   ├── Dockerfile.isaac_ros              # Isaac ROS with TensorRT
│   ├── docker-compose.yml                # Multi-container setup
│   └── entrypoint.sh
│
├── scripts/                               # Workspace-level utilities
│   ├── setup_workspace.sh                # One-command workspace build
│   ├── flash_jetson.sh                   # Jetson Orin Nano setup script
│   ├── deploy_to_robot.sh                # Push code to robot
│   └── run_field_test.sh                 # Launch + logging + monitoring
│
├── docs/                                  # Documentation
│   ├── tank_plan.txt                     # Master plan (copied from root)
│   ├── workspace_structure.md            # This file
│   ├── hardware_setup.md                 # Mount diagrams, wiring
│   ├── calibration_guide.md              # Extrinsics, camera calibration
│   ├── tuning_guide.md                   # Parameter tuning procedures
│   └── field_test_protocol.md            # T1/T2/T3 test procedures
│
├── .github/                               # GitHub workflows
│   └── workflows/
│       ├── ci_build.yml                  # Build on push
│       ├── static_analysis.yml           # Linting, cppcheck
│       └── deploy.yml                    # Auto-deploy to robot (optional)
│
├── .gitignore                            # ROS2 + Python + build artifacts
├── .gitmodules                           # Submodule references
├── README.md                             # Quick start guide
└── LICENSE
```

---

## Package Dependency Graph

```
tank_bringup (top-level orchestrator)
  ├── tank_sensors (hardware interfaces)
  ├── tank_localization (Point-LIO + GNSS)
  │     └── depends: tank_msgs, tank_sensors
  ├── tank_perception (LiDAR + camera → costmap)
  │     └── depends: tank_msgs, tank_sensors, isaac_ros (optional)
  ├── tank_navigation (planning)
  │     └── depends: tank_msgs, tank_perception, tank_localization
  ├── tank_control (ODrive + safety)
  │     └── depends: tank_msgs, tank_localization
  └── tank_utils (logging/monitoring)
        └── depends: tank_msgs
```

---

## Git Repository Structure

### Main Repository: `tank_autonomous_nav`
```
git@github.com:YourUsername/tank_autonomous_nav.git
```

**Branches:**
- `main` - stable, tested releases
- `develop` - active development
- `feature/*` - individual features (e.g., `feature/camera-segmentation`)
- `test/*` - field test branches (e.g., `test/T2-moderate-terrain`)
- `config/*` - parameter tuning branches (e.g., `config/rugged-terrain-v01`)

**Submodules:**
```bash
# In tank_ws/src/external/
git submodule add https://github.com/dfloreaa/point_lio_ros2.git
git submodule add https://github.com/unitreerobotics/unilidar_sdk2.git
git submodule add https://github.com/url-kaist/patchwork-plusplus.git
git submodule add https://github.com/aussierobots/ublox_dgnss.git
git submodule add -b humble-fw-v0.5.3 https://github.com/Factor-Robotics/odrive_ros2_control.git
# OR use custom fork for ODrive fw v0.5.6
```

---

## Development Workflow

### 1. Initial Setup (One-time)

```bash
# Clone repository
git clone --recursive git@github.com:YourUsername/tank_autonomous_nav.git
cd tank_autonomous_nav/tank_ws

# Run setup script
../scripts/setup_workspace.sh

# This script will:
# - Install ROS2 Humble dependencies
# - Build all packages
# - Set up Isaac ROS Docker environment
# - Download pre-trained models
# - Configure udev rules for sensors
```

### 2. Daily Development

```bash
# Update submodules
git submodule update --remote

# Build specific package
cd tank_ws
colcon build --packages-select tank_perception --symlink-install

# Source workspace
source install/setup.bash

# Launch for testing
ros2 launch tank_bringup teleop.launch.py
```

### 3. Field Testing

```bash
# Start full system + logging
./scripts/run_field_test.sh --test T2 --duration 30min

# This script launches:
# - Full navigation stack
# - Rosbag recording (sensors, odometry, commands)
# - KPI monitoring dashboard
# - Safety watchdog
```

### 4. Parameter Tuning

```bash
# Create tuning branch
git checkout -b config/slope-tuning-v02

# Edit parameters
vim tank_ws/src/tank_bringup/config/tank_params.yaml

# Test & commit
git add tank_ws/src/tank_bringup/config/
git commit -m "Adjust slope limits for 25° validation"
```

---

## Docker Setup (for Isaac ROS)

### Dockerfile Structure

```dockerfile
# docker/Dockerfile.isaac_ros
FROM nvcr.io/nvidia/l4t-jetpack:r36.4.3

# Install Isaac ROS dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-isaac-ros-dnn-inference \
    ros-humble-isaac-ros-image-proc \
    && rm -rf /var/lib/apt/lists/*

# Copy TensorRT models
COPY data/segmentation_training/segformer_b0_fp16.engine /models/

# Setup entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
```

### Docker Compose

```yaml
# docker/docker-compose.yml
version: '3.8'

services:
  isaac_ros:
    build:
      context: ..
      dockerfile: docker/Dockerfile.isaac_ros
    runtime: nvidia
    network_mode: host
    volumes:
      - ../tank_ws:/workspace/tank_ws
      - ../data:/workspace/data
    devices:
      - /dev/video0:/dev/video0  # Camera
    environment:
      - ROS_DOMAIN_ID=0
    command: ros2 launch tank_perception camera_segmentation.launch.py

  navigation:
    build:
      context: ..
      dockerfile: docker/Dockerfile.ros2
    network_mode: host
    volumes:
      - ../tank_ws:/workspace/tank_ws
    depends_on:
      - isaac_ros
    command: ros2 launch tank_bringup navigation.launch.py
```

---

## Build & Deployment Scripts

### `scripts/setup_workspace.sh`

```bash
#!/bin/bash
set -e

echo "=== Tank Autonomous Nav - Workspace Setup ==="

# 1. Install ROS2 Humble
if ! command -v ros2 &> /dev/null; then
    echo "Installing ROS2 Humble..."
    # Add ROS2 apt repository & install
    sudo apt update && sudo apt install -y ros-humble-desktop
fi

# 2. Install dependencies
echo "Installing dependencies..."
sudo apt install -y \
    ros-humble-nav2-bringup \
    ros-humble-robot-localization \
    libeigen3-dev \
    python3-colcon-common-extensions

# 3. Build workspace
echo "Building tank_ws..."
cd tank_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 4. Setup Isaac ROS (Docker)
echo "Setting up Isaac ROS Docker environment..."
cd ../isaac_ros_ws
# Run Isaac ROS setup (from NVIDIA docs)

# 5. Download models
echo "Downloading pre-trained models..."
# wget or gsutil download for SegFormer-B0

echo "=== Setup Complete ==="
echo "Source workspace: source tank_ws/install/setup.bash"
```

### `scripts/deploy_to_robot.sh`

```bash
#!/bin/bash
# Deploy code to Jetson Orin Nano on robot

ROBOT_IP="192.168.1.100"  # Update with your robot's IP
ROBOT_USER="tank"

echo "Deploying to robot at $ROBOT_IP..."

# Sync workspace
rsync -avz --exclude 'build' --exclude 'install' --exclude 'log' \
    tank_ws/ $ROBOT_USER@$ROBOT_IP:~/tank_ws/

# Rebuild on robot
ssh $ROBOT_USER@$ROBOT_IP "cd ~/tank_ws && colcon build --symlink-install"

echo "Deployment complete."
```

---

## Configuration Management

### Parameter Organization

**tank_ws/src/tank_bringup/config/tank_params.yaml**

```yaml
# Master parameter file - includes all subsystem configs

tank:
  # Platform specs
  platform:
    track_width: 0.6
    contact_length: 0.41
    cg_height: 0.229
    max_speed: 1.5
    max_angular_velocity: 0.8

  # Safety limits
  safety:
    pitch_warning: 28.0
    pitch_critical: 32.0
    pitch_emergency: 38.0
    motor_temp_limit: 80.0
    slip_threshold: 0.15

  # Navigation
  navigation:
    costmap_radius: 25.0
    resolution: 0.15
    inflation_radius: 0.45
    corridor_width: 4.5
    dwa_vx_samples: 15
    dwa_vtheta_samples: 25

  # Perception
  perception:
    slope_soft_limit: 25.0
    slope_hard_limit: 30.0
    roughness_soft: 0.06
    roughness_hard: 0.10
    camera_alpha: 0.85
```

### Version-Controlled Configs

```bash
# Save tested parameter set
git tag -a params-v1.2-rugged "Tuned for 25° slopes, high roughness"
git push --tags

# Load specific parameter version
git checkout params-v1.2-rugged -- tank_ws/src/tank_bringup/config/
```

---

## Data Management

### Rosbag Recording Strategy

```bash
# Automated recording in scripts/run_field_test.sh
ros2 bag record \
  /lidar_front/pointcloud \
  /lidar_rear/pointcloud \
  /camera/image_raw \
  /camera/segmentation \
  /gnss/fix \
  /odom \
  /cmd_vel \
  /safety/status \
  /diagnostics \
  -o data/rosbags/T2_$(date +%Y%m%d_%H%M%S)
```

### Training Data Collection

```bash
# Extract camera frames for labeling
ros2 run tank_utils extract_frames.py \
  --bag data/rosbags/T2_20250101_143022 \
  --topic /camera/image_raw \
  --output data/segmentation_training/images/ \
  --interval 2.0  # Extract every 2 seconds
```

---

## CI/CD Pipeline

### `.github/workflows/ci_build.yml`

```yaml
name: Build & Test

on: [push, pull_request]

jobs:
  build:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v3
        with:
          submodules: recursive

      - name: Install ROS2 Humble
        run: |
          sudo apt update
          sudo apt install -y ros-humble-desktop

      - name: Build workspace
        run: |
          cd tank_ws
          source /opt/ros/humble/setup.bash
          colcon build --packages-up-to tank_bringup

      - name: Run tests
        run: |
          source tank_ws/install/setup.bash
          colcon test --packages-select tank_*
```

---

## ODrive Integration Notes

### Firmware Compatibility Issue

**Your Setup:** ODrive firmware v0.5.6  
**Official Support:** ROS2 Humble branches only support fw v0.5.3 and v0.5.1

### Solution Options

#### Option 1: Downgrade Firmware (Safest)
```bash
# Flash ODrive to v0.5.3 to match official ROS2 Humble branch
odrivetool dfu odrive-v3.6-56V-fw-v0.5.3.hex

# Then use official repo
git submodule add -b humble-fw-v0.5.3 \
  https://github.com/Factor-Robotics/odrive_ros2_control.git \
  tank_ws/src/external/odrive_ros2_control
```

#### Option 2: Fork & Adapt PR #46 (Stay on v0.5.6)
```bash
# Fork the official repo to your GitHub account
gh repo fork Factor-Robotics/odrive_ros2_control

# Clone your fork
git clone -b humble-fw-v0.5.3 \
  https://github.com/YourUsername/odrive_ros2_control.git \
  tank_ws/src/external/odrive_ros2_control

cd tank_ws/src/external/odrive_ros2_control

# Create branch for your firmware
git checkout -b humble-fw-v0.5.6

# Apply changes from PR #46 (minimal CMakeLists.txt updates)
# URL: https://github.com/Factor-Robotics/odrive_ros2_control/pull/46/files

# Test with your hardware and push to your fork
git push origin humble-fw-v0.5.6
```

#### Option 3: Test Compatibility (Quick Validation)
```bash
# Try humble-fw-v0.5.3 branch with your v0.5.6 firmware
git submodule add -b humble-fw-v0.5.3 \
  https://github.com/Factor-Robotics/odrive_ros2_control.git \
  tank_ws/src/external/odrive_ros2_control

# Build and test
cd tank_ws
colcon build --packages-select odrive_hardware_interface
source install/setup.bash

# Run basic test
ros2 launch odrive_demo_bringup diffbot.launch.py

# Monitor for errors - if protocol mismatch occurs, go to Option 1 or 2
```

### ODrive Protocol References

- [v0.5.6 Getting Started](https://docs.odriverobotics.com/v/0.5.6/getting-started.html)
- [v0.5.6 Native Protocol](https://docs.odriverobotics.com/v/0.5.6/native-protocol.html)
- [v0.5.6 CAN Protocol](https://docs.odriverobotics.com/v/0.5.6/can-protocol.html)
- [ODrive Types Reference](https://docs.odriverobotics.com/v/0.5.6/fibre_types/com_odriverobotics_ODrive.html)

### Known Changes Between v0.5.3 → v0.5.6
*(Check ODrive release notes for breaking changes)*

If protocol differences exist, you'll need to update:
- `odrive_hardware_interface/src/odrive_hardware_interface.cpp` (protocol calls)
- `odrive_hardware_interface/include/odrive_hardware_interface/odrive.hpp` (API definitions)

### Integration Testing Checklist

Before using with tank:
1. [ ] ODrive responds to odrivetool commands
2. [ ] ROS2 node discovers ODrive (check USB permissions: `sudo chmod 666 /dev/ttyUSB*`)
3. [ ] Encoder reading works (verify counts increase with wheel rotation)
4. [ ] Velocity command works (test with `ros2 topic pub /cmd_vel`)
5. [ ] Watchdog triggers properly (stop sending commands → motors stop)
6. [ ] Error states reported correctly (disconnect motor → check diagnostics)
7. [ ] Temperature/voltage monitoring works

---

## CAN Bus vs USB for ODrive Control

### Your Hardware
- **Jetson Orin Nano:** Native CAN FD support (up to 5 Mbps)
- **Adafruit CAN Pal (TJA1051T/3):** Transceiver already installed
- **ODrive firmware v0.5.6:** Supports both USB and CAN protocols

### Critical Finding
**The Factor-Robotics/odrive_ros2_control package does NOT support CAN yet!** (listed in TODO)
- ✅ USB native protocol: DONE
- ❌ CAN protocol: TODO

**If you use CAN, you must write a custom ROS2 package (~1 week effort).**

### USB vs CAN Comparison

| Factor | USB (Native Protocol) | CAN Bus |
|--------|----------------------|---------|
| **Existing ROS2 support** | ✅ Yes (Factor-Robotics package) | ❌ No (must write custom) |
| **Firmware compatibility** | ⚠️ v0.5.3/v0.5.1 official, v0.5.6 untested | ✅ Protocol stable across versions |
| **Bandwidth** | ✅ ~10 Mbps | ⚠️ 1 Mbps (CAN 2.0) or 5 Mbps (CAN FD) |
| **Latency** | ✅ Lower (~1-2ms) | ⚠️ Moderate (~5-10ms) |
| **Noise immunity** | ❌ Poor (twisted pair, not differential) | ✅ **Excellent** (differential signaling) |
| **Vibration resistance** | ❌ USB connectors can loosen | ✅ Screw terminals on CAN Pal |
| **Cable length** | ❌ <5m typical | ✅ Up to 40m @ 1 Mbps |
| **Multi-device support** | ❌ Need USB hub, separate cables | ✅ Daisy-chain on single bus |
| **Real-time determinism** | ❌ Not guaranteed | ✅ CAN arbitration ensures delivery |
| **Wiring complexity** | ✅ Simple point-to-point | ⚠️ Requires termination resistors |
| **Setup complexity** | ✅ Plug and play | ⚠️ Configure bitrate, CAN IDs |
| **Implementation effort** | ✅ Use existing package (if compatible) | ❌ ~1 week to write custom node |

### Recommendation

**For tracked off-road vehicle → Use CAN Bus**

**Why:**
1. **Vibration/shock resistance** is critical on tracked platform
2. **EMI immunity** - motors generate electrical noise
3. **Deterministic timing** - safety watchdog requires reliable delivery
4. **Future expansion** - easy to add more ODrives or CAN devices
5. **You already have the CAN transceiver installed**

**The 1-week implementation cost is worth it for robustness.**

---

## Testing Jetson CAN Interface

### Step 1: Enable CAN on Jetson
```bash
# Load CAN kernel modules
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan  # Jetson-specific NVIDIA driver

# Verify can0 interface exists
ip link show can0
# Should show: "can0: <NOARP,ECHO> mtu 16 qdisc noop state DOWN"
```

### Step 2: Configure CAN Bitrate
```bash
# Set bitrate to 500 kbps (typical for ODrive)
sudo ip link set can0 type can bitrate 500000

# Enable CAN FD mode (optional, if ODrive supports it)
sudo ip link set can0 type can bitrate 500000 dbitrate 2000000 fd on

# Bring interface up
sudo ip link set up can0

# Verify
ip -details link show can0
# Should show: "state ERROR-ACTIVE" (good) or "state BUS-OFF" (bad - wiring issue)
```

### Step 3: Install CAN Utilities
```bash
sudo apt update
sudo apt install can-utils
```

### Step 4: Test CAN Communication

#### Without ODrive (loopback test)
```bash
# Terminal 1: Listen for CAN messages
candump can0

# Terminal 2: Send test message
cansend can0 123#DEADBEEF

# Should see message in Terminal 1
```

#### With ODrive Connected

**ODrive Wiring:**
```
Adafruit CAN Pal → ODrive
────────────────────────────
CANH → ODrive CAN_H (white wire typical)
CANL → ODrive CAN_L (blue wire typical)
GND  → ODrive GND
```

**Enable 120Ω termination on CAN Pal** (flip switch to ON if it's end of bus)

**Configure ODrive for CAN:**
```bash
# Using odrivetool over USB
odrivetool

# In odrivetool:
odrv0.config.enable_can_a = True
odrv0.axis0.config.can_node_id = 0  # Axis 0 = left motor
odrv0.axis1.config.can_node_id = 1  # Axis 1 = right motor
odrv0.can.config.baud_rate = 500000
odrv0.save_configuration()
odrv0.reboot()
```

**Test CAN messages:**
```bash
# Listen for ODrive heartbeat (sent every 100ms)
candump can0

# You should see messages like:
# can0  001   [8]  XX XX XX XX XX XX XX XX
# can0  021   [8]  XX XX XX XX XX XX XX XX
# (0x001 = axis 0 heartbeat, 0x021 = axis 1 heartbeat)
```

**Send velocity command:**
```bash
# Set axis 0 to 1.0 rad/s velocity
# CAN ID = 0x00B | (axis_id << 5) = 0x00B for axis 0
# Data = velocity (float32) + torque_feedforward (float32)

# Using Python to pack the message:
python3 << EOF
import struct
import os
vel = 1.0
torque_ff = 0.0
data = struct.pack('<ff', vel, torque_ff)
cmd = f"cansend can0 00B#{data.hex().upper()}"
os.system(cmd)
EOF
```

### Step 5: Persistent CAN Configuration

Create `/etc/systemd/network/80-can.network`:
```ini
[Match]
Name=can0

[CAN]
BitRate=500000
# Uncomment for CAN FD:
# DataBitRate=2000000
# FDMode=yes
```

Enable:
```bash
sudo systemctl enable systemd-networkd
sudo systemctl start systemd-networkd
```

---

## Custom CAN Package Implementation

If CAN tests succeed, implement `tank_odrive_can` package:

### Minimal Working Example

**File: `src/tank_odrive_can/src/odrive_can_node.cpp`**

```cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <cstring>

class ODriveCANNode : public rclcpp::Node {
public:
    ODriveCANNode() : Node("odrive_can_node") {
        // Open CAN socket
        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        struct ifreq ifr;
        strcpy(ifr.ifr_name, "can0");
        ioctl(can_socket_, SIOCGIFINDEX, &ifr);
        
        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_index;
        bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr));
        
        // Subscribe to cmd_vel
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&ODriveCANNode::cmdVelCallback, this, std::placeholders::_1));
        
        // Publish odometry
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        
        // Timer to read CAN messages
        timer_ = create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&ODriveCANNode::readCANMessages, this));
    }
    
private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Convert twist to differential drive velocities
        float v_linear = msg->linear.x;
        float v_angular = msg->angular.z;
        float track_width = 0.6;  // 600mm
        
        float v_left = v_linear - (v_angular * track_width / 2.0);
        float v_right = v_linear + (v_angular * track_width / 2.0);
        
        // Send CAN messages (ODrive Simple CAN Protocol)
        sendVelocityCommand(0, v_left, 0.0);   // Axis 0 = left
        sendVelocityCommand(1, v_right, 0.0);  // Axis 1 = right
    }
    
    void sendVelocityCommand(uint8_t axis_id, float velocity, float torque_ff) {
        struct can_frame frame;
        frame.can_id = 0x00B | (axis_id << 5);  // Set_Input_Vel command
        frame.can_dlc = 8;
        
        memcpy(&frame.data[0], &velocity, sizeof(float));
        memcpy(&frame.data[4], &torque_ff, sizeof(float));
        
        write(can_socket_, &frame, sizeof(frame));
    }
    
    void readCANMessages() {
        struct can_frame frame;
        int nbytes = read(can_socket_, &frame, sizeof(frame));
        
        if (nbytes > 0) {
            // Parse encoder feedback (CAN ID 0x009 | (axis_id << 5))
            if ((frame.can_id & 0x1F) == 0x009) {
                uint8_t axis_id = (frame.can_id >> 5) & 0x1F;
                float position, velocity;
                memcpy(&position, &frame.data[0], sizeof(float));
                memcpy(&velocity, &frame.data[4], sizeof(float));
                
                // Update odometry (combine left/right wheels)
                // ... odometry calculation ...
            }
        }
    }
    
    int can_socket_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ODriveCANNode>());
    rclcpp::shutdown();
    return 0;
}
```

### ODrive CAN Protocol Reference

From [ODrive CAN docs](https://docs.odriverobotics.com/v/0.5.6/can-protocol.html):

| Command | CAN ID | Data | Description |
|---------|--------|------|-------------|
| Heartbeat | `0x001 \| (axis << 5)` | 8 bytes | Axis state, error flags |
| Set Velocity | `0x00B \| (axis << 5)` | vel (float32) + torque_ff (float32) | Velocity setpoint + feedforward |
| Get Encoder | `0x009 \| (axis << 5)` | pos (float32) + vel (float32) | Position & velocity feedback |
| Set Position | `0x00A \| (axis << 5)` | pos (float32) + vel_ff (int16) + torque_ff (int16) | Position control |
| Estop | `0x002 \| (axis << 5)` | None | Emergency stop |

### Build & Test

```bash
cd tank_ws
colcon build --packages-select tank_odrive_can
source install/setup.bash

# Launch
ros2 launch tank_odrive_can odrive_can.launch.py

# Test with teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Unitree L2 LiDAR Setup (unilidar_sdk2)

### Hardware Specifications

From [Unitree L2 product page](https://github.com/unitreerobotics/unilidar_sdk2):

| Spec | Value |
|------|-------|
| **Range** | 30m @ 90% reflectivity, 15m @ 10% reflectivity |
| **FOV** | 360° × 96° (full panoramic) |
| **Point rate** | 64,000 pts/s effective (128k sampling) |
| **Scan frequency** | 5.55 Hz (customizable) |
| **Accuracy** | ≤2.0cm |
| **Near-field blind zone** | 0.05m (excellent for obstacle detection) |
| **IMU** | 6-axis (3-axis accel + 3-axis gyro) - **hardware time-synced!** |
| **Communication** | Ethernet UDP (default) or TTL UART |
| **Power** | 10W |
| **Weight** | 230g |

### Installation

```bash
cd tank_ws/src/external

# Clone unilidar_sdk2
git clone https://github.com/unitreerobotics/unilidar_sdk2.git

# Build (ROS2)
cd unilidar_sdk2/unitree_lidar_ros2
colcon build

# OR if using Point-LIO (it has L2 support built-in)
cd tank_ws/src/external
git clone https://github.com/dfloreaa/point_lio_ros2.git

cd ../..
colcon build --packages-select point_lio
```

### Configuration

**Default L2 settings (Ethernet UDP mode):**
- IP Address: `192.168.123.123` (LiDAR default)
- Port: `18888`
- Host IP: `192.168.123.30` (configure your PC)
- Topics:
  - Point cloud: `/unilidar/cloud`
  - IMU: `/unilidar/imu`
  - Frame ID: `unilidar_lidar` (cloud), `unilidar_imu` (IMU)

**To change L2 IP or communication mode:**
Use Unitree's host computer software or refer to [L2 SDK docs](https://support.unitree.com/home/en/L2_SDK/L2_develope).

### Testing L2 Alone (Without Point-LIO)

```bash
# Launch standalone L2 driver
source tank_ws/src/external/unilidar_sdk2/unitree_lidar_ros2/install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py

# Check topics
ros2 topic list | grep unilidar

# Echo point cloud
ros2 topic echo /unilidar/cloud --no-arr

# Echo IMU
ros2 topic echo /unilidar/imu
# Should show:
#   - orientation (quaternion)
#   - angular_velocity (x, y, z)
#   - linear_acceleration (x, y, z)

# Visualize in RViz
rviz2
# Add PointCloud2, subscribe to /unilidar/cloud
# Set Fixed Frame to "unilidar_lidar"
```

### Point-LIO + L2 Integration (Recommended)

**Point-LIO ROS2 has official L2 support** with pre-configured launch file!

**File: Already exists in point_lio_ros2**
```bash
# Launch Point-LIO with L2
source tank_ws/src/external/point_lio_ros2/install/setup.bash
ros2 launch point_lio mapping_unilidar_l2.launch.py

# This automatically:
# - Subscribes to /unilidar/cloud and /unilidar/imu
# - Configures extrinsics (identity - IMU is in LiDAR frame)
# - Publishes /point_lio/odom (odometry)
# - Publishes /point_lio/cloud (registered point cloud)
```

**Verify Point-LIO is working:**
```bash
# Check Point-LIO odometry output
ros2 topic echo /point_lio/odom

# Should see:
#   pose.position (x, y, z) - increments as you move LiDAR
#   twist.linear (vx, vy, vz) - velocity
#   covariance - position uncertainty

# Check drift
# Move LiDAR 1m forward, check if pose.position.x ≈ 1.0
# Typical drift: <1% (should be 0.99-1.01m)
```

### Dual L2 Setup (Front + Rear)

**Your tank has TWO L2 units.** Configuration:

**Front L2:**
- Default IP: `192.168.123.123` (keep default)
- Use for Point-LIO localization
- Topic remapping: `/unilidar/cloud` → `/lidar_front/pointcloud`

**Rear L2:**
- Change IP to: `192.168.123.124` (use Unitree host computer)
- Use for rear perception/costmap only
- Topic remapping: `/unilidar/cloud` → `/lidar_rear/pointcloud`

**Launch file for dual L2:**

**File: `tank_ws/src/tank_sensors/launch/lidar_dual.launch.py`**

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Front L2 (for Point-LIO)
        Node(
            package='unitree_lidar_ros2',
            executable='unitree_lidar_ros2_node',
            name='lidar_front',
            parameters=[{
                'lidar_ip': '192.168.123.123',
                'port': 18888,
                'frame_id': 'lidar_front'
            }],
            remappings=[
                ('/unilidar/cloud', '/lidar_front/pointcloud'),
                ('/unilidar/imu', '/lidar_front/imu')
            ]
        ),
        
        # Rear L2 (for perception only)
        Node(
            package='unitree_lidar_ros2',
            executable='unitree_lidar_ros2_node',
            name='lidar_rear',
            parameters=[{
                'lidar_ip': '192.168.123.124',  # Different IP!
                'port': 18888,
                'frame_id': 'lidar_rear'
            }],
            remappings=[
                ('/unilidar/cloud', '/lidar_rear/pointcloud'),
                ('/unilidar/imu', '/lidar_rear/imu')  # Not used, but published
            ]
        ),
        
        # Point-LIO (subscribes to front L2)
        Node(
            package='point_lio',
            executable='point_lio_node',
            name='point_lio',
            parameters=[{
                'lidar_topic': '/lidar_front/pointcloud',
                'imu_topic': '/lidar_front/imu',
                # Extrinsics (identity - IMU in LiDAR frame)
                'extrinsic_T': [0.0, 0.0, 0.0],
                'extrinsic_R': [1.0, 0.0, 0.0,
                                0.0, 1.0, 0.0,
                                0.0, 0.0, 1.0]
            }]
        )
    ])
```

### IMU Configuration for Point-LIO

From L2 specs, we know:
- **6-axis IMU** (accel + gyro only, no magnetometer)
- **Hardware time-synchronized** with LiDAR

**Point-LIO config (update if needed):**

```yaml
# In point_lio config (usually auto-configured for L2)
imu:
  acc_norm: 9.81  # Gravity (m/s²)
  
  # Saturation limits (estimate - verify from Unitree support)
  satu_acc: 16.0   # Accelerometer saturation (m/s²) - typical for MEMS IMU
  satu_gyro: 17.5  # Gyroscope saturation (rad/s) - typical for MEMS IMU
  
  # Note: L2 IMU is likely consumer-grade MEMS
  # If you experience saturation warnings on 30° slopes, increase these limits
```

**To find exact saturation limits:**
1. Check [Unitree L2 support docs](https://support.unitree.com/home/en/L2_SDK/L2_Overview_DATA)
2. OR test empirically: shake L2 hard and monitor IMU topic
3. OR contact Unitree support for IMU datasheet

### L2 Mounting Guidelines (From Plan)

**Front L2:**
- Height: ~50 cm AGL (above ground level)
- Orientation: Level (0° tilt)
- Position: At/just ahead of nose
- Goal: See ground from ~0.3-0.5m beyond front bumper

**Rear L2:**
- Height: 45-50 cm AGL
- Orientation: Level (0° tilt)
- Position: At/just behind tail
- Goal: See ground from ~0.3-0.5m beyond rear bumper

**Vibration isolation:**
- Use 45-55A durometer rubber standoffs
- Critical for tracked vehicle (high vibration)

**Cable management:**
- Ethernet cable: <5m recommended (UDP packet loss increases beyond)
- Route away from motor power cables (EMI)

### Troubleshooting

**No point cloud data:**
```bash
# Check if L2 is reachable
ping 192.168.123.123

# Check network interface
ip addr show
# Should have 192.168.123.30 (or similar)

# If not, configure:
sudo ip addr add 192.168.123.30/24 dev eth0  # Replace eth0 with your interface
```

**IMU data looks wrong:**
```bash
# Check IMU calibration
# L2 should be stationary and level
ros2 topic echo /unilidar/imu

# linear_acceleration.z should be ~9.8 m/s² (gravity)
# angular_velocity should be ~0 when stationary

# If not, L2 may need calibration (contact Unitree support)
```

**Point-LIO drift is high (>1%):**
- Check L2 mount - must be rigid (no vibration dampening issues)
- Verify IMU saturation limits are correct
- Ensure L2 has clear view (no obstruction within 0.3m)
- Check for loose mount screws

**Dual L2 not working:**
- Verify both L2s have different IP addresses
- Check each L2 individually first
- Ensure switch/router can handle both streams simultaneously

---

## ZED-F9P GNSS Setup (aussierobots/ublox_dgnss)

### Why This Driver

[aussierobots/ublox_dgnss](https://github.com/aussierobots/ublox_dgnss) is the best ROS2 driver for ZED-F9P:
- ✅ Native UBX protocol (not just NMEA)
- ✅ High-precision position data (`/ubx_nav_hp_pos_llh`)
- ✅ Proper covariance publishing for EKF fusion
- ✅ NTRIP client included (for future RTK upgrade)
- ✅ ROS2 Humble support
- ✅ Standard GNSS works out-of-box (no RTK required)

### Installation

```bash
cd tank_ws/src/external
git clone https://github.com/aussierobots/ublox_dgnss.git

cd ../..
colcon build --packages-select ublox_ubx_msgs ublox_ubx_interfaces \
  ublox_dgnss ublox_dgnss_node ublox_nav_sat_fix_hp_node ntrip_client_node
```

### Configuration

**File: `tank_ws/src/tank_sensors/config/gnss_f9p.yaml`**

```yaml
ublox_dgnss_node:
  ros__parameters:
    device: /dev/ttyACM0          # USB connection (change if needed)
    
    # Frame IDs
    frame_id: gnss
    
    # UBX message output configuration (enable what you need)
    CFG_MSGOUT_UBX_NAV_PVT_USB: 1           # Position/Velocity/Time
    CFG_MSGOUT_UBX_NAV_HP_POS_LLH_USB: 1    # High-precision Lat/Lon/Height
    CFG_MSGOUT_UBX_NAV_COV_USB: 1           # Covariance (for EKF)
    CFG_MSGOUT_UBX_NAV_STATUS_USB: 1        # Fix quality status
    CFG_MSGOUT_UBX_NAV_DOP_USB: 1           # Dilution of Precision
    CFG_MSGOUT_UBX_NAV_SAT_USB: 1           # Satellite info
    
    # Update rate (Hz)
    CFG_RATE_MEAS: 200            # 5 Hz measurement rate (200ms)
    
    # GNSS systems (enable all for best accuracy without RTK)
    CFG_SIGNAL_GPS_ENA: 1
    CFG_SIGNAL_GAL_ENA: 1         # Galileo
    CFG_SIGNAL_BDS_ENA: 1         # BeiDou
    CFG_SIGNAL_QZSS_ENA: 1        # QZSS (Japan)
    
    # Logging
    debug: false                   # Set true for verbose output
```

### Launch File

**File: `tank_ws/src/tank_sensors/launch/gnss.launch.py`**

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Load config
    config = os.path.join(
        get_package_share_directory('tank_sensors'),
        'config',
        'gnss_f9p.yaml'
    )
    
    return LaunchDescription([
        # Main GNSS node
        Node(
            package='ublox_dgnss_node',
            executable='ublox_dgnss_node',
            name='ublox_dgnss_node',
            output='screen',
            parameters=[config]
        ),
        
        # NavSatFix publisher (for nav2 compatibility)
        Node(
            package='ublox_nav_sat_fix_hp_node',
            executable='ublox_nav_sat_fix_hp_node',
            name='ublox_nav_sat_fix_hp_node',
            output='screen',
            parameters=[{'frame_id': 'gnss'}],
            remappings=[
                ('/fix', '/gnss/fix')  # Standard topic for EKF
            ]
        )
    ])
```

### Testing GNSS

```bash
# Launch GNSS driver
ros2 launch tank_sensors gnss.launch.py

# Check topics
ros2 topic list | grep gnss

# Monitor high-precision position
ros2 topic echo /ubx_nav_hp_pos_llh

# Check fix quality
ros2 topic echo /ubx_nav_status
# Look for gps_fix.fix_type: 3 (3D fix)
# diff_soln: true if differential correction active

# Check NavSatFix output (for EKF)
ros2 topic echo /gnss/fix
# Should have status.status: 0 (FIX)
# position_covariance should be populated
```

### Expected Accuracy (No RTK)

With standard GNSS (no RTK corrections):
- **Horizontal accuracy:** 2-5m typical (open sky)
- **Vertical accuracy:** 5-10m typical
- **Under tree cover:** 5-15m (degrades)
- **Urban canyon:** >15m or no fix

From `/ubx_nav_hp_pos_llh`:
```yaml
h_acc: 2500   # 2.5m horizontal accuracy (in mm, scale by 0.1)
v_acc: 5000   # 5.0m vertical accuracy
```

### Checking HDOP (Dilution of Precision)

```bash
ros2 topic echo /ubx_nav_dop

# Good fix:
#   g_dop: <5   (overall)
#   h_dop: <3   (horizontal) - this is what we use for corridor adaptation
#   v_dop: <5   (vertical)
```

**Corridor width adaptation in global planner:**
- HDOP < 2.0 → corridor = 4m (good fix)
- HDOP 2.0-3.0 → corridor = 5m
- HDOP > 3.0 → corridor = 6-7m (poor fix)

### Device Identification

If you have multiple USB devices, identify ZED-F9P by serial:

```bash
# List USB devices
ls -l /dev/serial/by-id/

# Should show something like:
# usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00
```

Update config:
```yaml
device: /dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00
```

### Integration with EKF (robot_localization)

The NavSatFix output (`/gnss/fix`) can be fused with Point-LIO:

**File: `tank_ws/src/tank_localization/config/gnss_fusion.yaml`**

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 10.0
    
    # Input sources
    odom0: /point_lio/odom      # Point-LIO odometry
    odom0_config: [false, false, false,   # x, y, z
                   false, false, false,   # roll, pitch, yaw
                   true,  true,  false,   # vx, vy, vz
                   false, false, false,   # vroll, vpitch, vyaw
                   false, false, false]   # ax, ay, az
    
    navsat0: /gnss/fix          # GNSS fix
    navsat0_config: [true,  true,  false,  # Use only x, y from GNSS
                     false, false, false,
                     false, false, false,
                     false, false, false,
                     false, false, false]
    
    # Output
    publish_tf: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: map
```

### Future: Adding RTK via NTRIP

If you want cm-level accuracy later, enable NTRIP:

```bash
# Install NTRIP node
colcon build --packages-select ntrip_client_node

# Launch with credentials
ros2 launch ublox_dgnss ntrip_client.launch.py \
  use_https:=true \
  host:=rtk2go.com \
  port:=2101 \
  mountpoint:=YOUR_MOUNTPOINT \
  username:=$NTRIP_USERNAME \
  password:=$NTRIP_PASSWORD

# ZED-F9P will automatically use RTCM corrections
# Accuracy improves to 1-5cm horizontal
```

---

## Quick Start Commands

```bash
# 1. Clone & build
git clone --recursive https://github.com/YourUsername/tank_autonomous_nav.git
cd tank_autonomous_nav
./scripts/setup_workspace.sh

# 2. Launch simulation (if available)
source tank_ws/install/setup.bash
ros2 launch tank_bringup tank_full.launch.py use_sim:=true

# 3. Launch on real robot
ros2 launch tank_bringup tank_full.launch.py

# 4. Teleoperate for testing
ros2 launch tank_bringup teleop.launch.py

# 5. Run autonomous mission
ros2 run tank_navigation waypoint_commander \
  --waypoints "[[10,0], [10,10], [0,10]]"
```

---

## Next Steps

1. **Create GitHub repository** - initialize with README, .gitignore, LICENSE
2. **Set up base workspace** - ROS2 Humble + basic package structure
3. **Add submodules** - Point-LIO, unilidar_sdk2, Patchwork++
4. **Implement Phase 1** (tank_control + tank_sensors) - get hardware talking
5. **Docker for Isaac ROS** - camera segmentation pipeline
6. **Iterate through phases** - localization → perception → navigation

Ready to start when you are!

