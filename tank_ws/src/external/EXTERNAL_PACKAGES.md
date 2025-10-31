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

# 4. Isaac ROS (Camera segmentation with TensorRT)
git clone --branch release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone --branch release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_inference.git
git clone --branch release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git

# (Optional for Phase 2+)
# 5. Patchwork++ (Ground extraction)
git clone https://github.com/url-kaist/patchwork-plusplus.git

# (Optional - Phase 2+) Isaac ROS Nvblox (3D reconstruction)
# git clone --branch release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git

# (Optional if using USB for ODrive instead of CAN)
# 6. ODrive ROS2 Control
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
#   isaac_ros_common/
#   isaac_ros_dnn_inference/
#   isaac_ros_image_pipeline/
```

## Build

```bash
cd ~/Tank_projects/tank_ws
source /opt/ros/humble/setup.bash

# Install dependencies
rosdep install --from-paths src/external --ignore-src -r -y

# Build all external packages
colcon build --packages-up-to \
  unitree_lidar_ros2 \
  point_lio \
  ublox_dgnss \
  isaac_ros_dnn_image_encoder \
  isaac_ros_tensor_rt \
  isaac_ros_image_proc \
  --symlink-install
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

### Isaac ROS Packages (Bare Metal Installation)

**Purpose:** Camera-based terrain segmentation using SegFormer-B0 with TensorRT acceleration

**Installation:**
```bash
cd ~/Tank_projects/tank_ws/src/external

# Core Isaac ROS packages for camera segmentation
git clone --branch release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone --branch release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_inference.git
git clone --branch release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git

# (Optional - Phase 2+) For 3D reconstruction and Isaac Sim export
# git clone --branch release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git
```

**What's Included:**

| Package | Purpose | Status |
|---------|---------|--------|
| `isaac_ros_common` | NITROS infrastructure, build utilities, core types | ✅ Installed |
| `isaac_ros_dnn_inference` | TensorRT inference for SegFormer-B0 | ✅ Installed |
| `isaac_ros_image_pipeline` | VPI-accelerated image preprocessing | ✅ Installed |
| `isaac_ros_nvblox` | Colored 3D reconstruction (LiDAR+Camera) | ⏸️ Phase 2 (after tank drives) |

**Build:**
```bash
cd ~/Tank_projects/tank_ws
source /opt/ros/humble/setup.bash

# Install Isaac ROS dependencies
sudo apt update
sudo apt install -y \
  ros-humble-isaac-ros-common \
  ros-humble-isaac-ros-nitros-bridge-ros2 \
  libvpi3 vpi3-dev \
  nvidia-tensorrt \
  python3-libnvinfer \
  python3-libnvinfer-dev

# Build Isaac ROS packages
colcon build --packages-up-to \
  isaac_ros_dnn_image_encoder \
  isaac_ros_tensor_rt \
  isaac_ros_image_proc
```

**Dependencies:**
- JetPack 6.2.1 (L4T 36.4.4) ✅
- CUDA 12.6 ✅
- TensorRT 10.3 ✅
- VPI 3.2 ✅

**Integration:**
- Subscribes to `/camera/image_raw` from e-CAM25
- Runs SegFormer-B0 TensorRT model for terrain classification
- Publishes semantic segmentation mask
- Used by costmap fusion for traversability

**Why Bare Metal (Not Docker):**
- Direct hardware access (camera, CAN, GNSS, LiDAR)
- Seamless integration with existing `tank_ws`
- Lower latency for real-time obstacle avoidance
- Simpler production deployment with systemd

**Isaac Sim Integration (Future):**
- Train SegFormer on real terrain data
- Test navigation behavior in simulation
- Phase 2: Use Nvblox to reconstruct real terrain → import to Isaac Sim

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

