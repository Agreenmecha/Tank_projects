# External ROS2 Packages & Hardware Setup

This directory contains external ROS2 packages (git cloned, not submodules) and hardware setup documentation.

## Why not Git Submodules?

- Easier development and modification
- No submodule update complications
- Cleaner workspace management

## Installation (Run on Jetson)

```bash
cd ~/Tank_projects/tank_ws/src/external

# 1. Point-LIO ROS2 (LiDAR-Inertial Odometry)
git clone https://github.com/dfloreaa/point_lio_ros2.git

# 2. Unitree L2 LiDAR SDK
git clone https://github.com/unitreerobotics/unilidar_sdk2.git

# 3. ublox DGNSS driver (ZED-F9P)
git clone https://github.com/aussierobots/ublox_dgnss.git

# (Optional for Phase 2+)
# 4. Patchwork++ (Ground extraction)
git clone https://github.com/url-kaist/patchwork-plusplus.git

# (Optional if using USB for ODrive instead of CAN)
# 5. ODrive ROS2 Control
git clone -b humble-fw-v0.5.3 https://github.com/Factor-Robotics/odrive_ros2_control.git
```

## Verify Installation

```bash
cd ~/Tank_projects/tank_ws/src/external
ls -la
# Should see:
#   point_lio_ros2/
#   unilidar_sdk2/
#   ublox_dgnss/
```

## Build

```bash
cd ~/Tank_projects/tank_ws
source /opt/ros/humble/setup.bash

# Build external packages
colcon build --packages-select unitree_lidar_ros2 point_lio ublox_dgnss --symlink-install
```

## Hardware Setup

### 0. e-CAM25_CUONX Camera
- **Purpose:** Global shutter camera for terrain segmentation
- **Setup:** See `ECAM25_CAMERA_SETUP.md` in this directory
- **Important:** Requires e-con Systems custom drivers (NOT generic v4l2)
- **Installation:** Run install script from e-con's release package
- **Package Location:** `~/Tank_projects/ecam-25docs/`
- **ROS2 Integration:** Uses `gscam` with `nvarguscamerasrc`

**Installation Summary:**
```bash
# Copy release package to Jetson
scp -r ~/Tank_projects/ecam-25docs/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_29-MAR-2025_R04 nvidia@jetson:~/

# On Jetson, run installer
cd ~/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_29-MAR-2025_R04
sudo ./install_binaries.sh
# Select 4-lane configuration
# System will reboot automatically

# Install gscam for ROS2
sudo apt install ros-humble-gscam
```

---

## Package Details

### 1. Point-LIO ROS2
- **Purpose:** LiDAR-Inertial Odometry for localization
- **Repo:** https://github.com/dfloreaa/point_lio_ros2
- **Features:** Official Unitree L2 support with `mapping_unilidar_l2.launch.py`
- **Config:** `tank_localization/config/point_lio_l2.yaml`

### 2. unilidar_sdk2
- **Purpose:** Unitree L2 LiDAR ROS2 driver
- **Repo:** https://github.com/unitreerobotics/unilidar_sdk2
- **Features:** Point cloud + IMU publishing, Ethernet UDP interface
- **Topics:** `/unilidar/cloud`, `/unilidar/imu`

### 3. ublox_dgnss
- **Purpose:** ZED-F9P GNSS driver
- **Repo:** https://github.com/aussierobots/ublox_dgnss
- **Features:** Native UBX protocol, HP position, covariance, no RTK required
- **Config:** `tank_sensors/config/gnss_f9p.yaml`

### 4. Patchwork++ (Phase 2)
- **Purpose:** Fast ground segmentation
- **Repo:** https://github.com/url-kaist/patchwork-plusplus
- **Use:** Remove ground points from LiDAR for obstacle detection

### 5. ODrive ROS2 Control (Optional)
- **Purpose:** ODrive motor control via USB
- **Repo:** https://github.com/Factor-Robotics/odrive_ros2_control
- **Branch:** `humble-fw-v0.5.3`
- **Note:** Only needed if using USB instead of CAN bus

## Updating Packages

```bash
cd ~/Tank_projects/tank_ws/src/external/<package_name>
git pull
cd ~/Tank_projects/tank_ws
colcon build --packages-select <package_name> --symlink-install
```

## Troubleshooting

**Problem:** Build fails with missing dependencies

**Solution:**
```bash
cd ~/Tank_projects/tank_ws
rosdep install --from-paths src/external --ignore-src -r -y
```

**Problem:** Point-LIO crashes on startup

**Solution:** Check Point-LIO config (`tank_localization/config/point_lio_l2.yaml`):
- Verify topic names match: `/lidar_front/pointcloud`, `/lidar_front/imu`
- Check extrinsics are correct for your L2 mount position

**Problem:** Unitree L2 not publishing data

**Solution:**
- Check L2 IP address: `ping 192.168.123.123`
- Verify Jetson network: `ip addr show` (should have 192.168.123.x address)
- Check L2 is powered on and Ethernet cable connected

**Problem:** GNSS not connecting

**Solution:**
- Check device: `ls /dev/ttyACM*`
- Verify permissions: `sudo chmod 666 /dev/ttyACM0`
- Check config device parameter matches actual device

