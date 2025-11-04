# Tank Autonomous Navigation System

Autonomous off-road navigation system for a tracked platform using dual 3D LiDARs, camera semantic segmentation, and Point-LIO localization.

**Target Capability:** 30° slope navigation through rugged terrain with obstacle avoidance and confident reversing.

---

## 📋 Project Documents

| Document | Description |
|----------|-------------|
| **[tank_plan.txt](tank_plan.txt)** | Master technical plan - hardware specs, algorithms, parameters, KPIs |
| **[workspace_structure.md](workspace_structure.md)** | ROS2 workspace architecture, package organization, build scripts |
| **[PROJECT_STATUS.md](PROJECT_STATUS.md)** | Current project status, completed items, open tasks, timeline |
| **[QUICK_REFERENCE.md](QUICK_REFERENCE.md)** | Field testing quick reference card |

### Setup & Installation Guides

| Document | Description |
|----------|-------------|
| **[freshstart/README.md](freshstart/README.md)** | Complete fresh installation guide for Jetson |
| **[LIDAR_SETUP_COMPLETE.md](LIDAR_SETUP_COMPLETE.md)** | Dual Unitree L2 LiDAR setup and configuration |
| **[GNSS_SETUP_COMPLETE.md](GNSS_SETUP_COMPLETE.md)** | u-blox ZED-F9P GNSS setup and integration |
| **[ODRIVE_JETSON_QUICKSTART.md](ODRIVE_JETSON_QUICKSTART.md)** | ODrive 0.5.4 installation and configuration |
| **[ECAM25_BUILD_OVERVIEW.md](ECAM25_BUILD_OVERVIEW.md)** | e-CAM25_CUONX camera driver build guide |
| **[FIELD_WIRELESS_SETUP.md](FIELD_WIRELESS_SETUP.md)** | Wireless access point configuration for field testing |
| **[REMOTE_VISUALIZATION.md](REMOTE_VISUALIZATION.md)** | Remote RViz2 setup for desktop visualization |

---

## 🛠️ Hardware Stack

### Platform
- **Tracked vehicle** (differential drive)
  - Track width: 600mm, Contact length: 410mm
  - CG height: 229mm (low center of gravity)
  - Rated: 100kg @ 6mph @ 15° slope
  - Typical payload: 30 lb (87% torque reserve)

### Compute
- **NVIDIA Jetson Orin Nano** (JetPack 6.2.1, L4T 36.4.7)
  - 68-73% CPU utilization (with Isaac ROS optimization)
  - 45-50% GPU utilization
  - Super Mode: 25W and uncapped MAXN SUPER modes available

### Sensors
- **2x Unitree L2 3D LiDAR** (front + rear)
  - Built-in 6-axis IMU (3-axis accel + gyro, hardware time-synced)
  - Front: Point-LIO localization (30m range, 64k pts/s)
  - Rear: Reverse navigation + 360° costmap
  - FOV: 360° × 96°, accuracy: ≤2cm
  - Near-field blind zone: 0.05m (excellent obstacle detection)
  
- **e-CAM25_CUONX** (AR0234 global shutter camera)
  - 1280x720 @ 20fps
  - Semantic segmentation for terrain classification
  
- **u-blox ZED-F9P GNSS** (standard GNSS with SBAS)
  - ~1-2cm accuracy with SBAS augmentation (tested)
  - Multi-band (L1/L2) for improved performance
  - 24/42 satellites tracked (excellent)
  - 5 Hz update rate via UBX protocol

### Actuators
- **ODrive 0.5.4 motor controller**
  - Velocity control mode
  - Watchdog timeout: 200ms
  - Wheel encoders for slip detection
  - USB interface with udev rules configured

---

## 🧠 Software Architecture

### Localization
- **Point-LIO ROS2** - LiDAR-inertial odometry using front L2 + built-in IMU
- **GNSS fusion** - EKF integration for long-term drift correction
- **Encoder odometry** - Cross-check for slip detection (15% threshold)

### Perception
- **Patchwork++** - Ground extraction (handles 30° slopes)
- **Isaac ROS + SegFormer-B0** - TensorRT-accelerated terrain segmentation (~10-15ms)
- **Costmap fusion** - LiDAR geometry + camera semantics → unified 360° costmap

### Planning & Control
- **DWA local planner** - Custom tracked vehicle kinematics (can pivot)
- **Global planner** - GNSS waypoints → corridors (4-5m width, adapts to fix quality)
- **Reverse manager** - Rear LiDAR-enabled backing with safety gates
- **Recovery behaviors** - Stuck detection, belly strike recovery, pivot maneuvers

### Safety
- **Pitch monitoring** - 28°/32°/38° warning/critical/emergency thresholds
- **Hardware E-stop** + software watchdog
- **Multi-sensor health monitoring** - LiDAR packet loss, camera confidence, GNSS quality, motor thermal
- **Degradation modes** - Graceful fallback (camera loss → LiDAR-only, etc.)

---

## 🎯 Key Parameters (Tuned for 600mm Track, 229mm CG, 30 lb payload)

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Max speed | 1.5 m/s | Conservative (56% of 2.7 m/s rating) |
| Slope soft limit | 25° | Start speed reduction |
| Slope hard limit | 30° | Absolute rejection (11.8° margin to 41.8° tip angle) |
| Inflation radius | 0.45m base, 0.50m on slopes | Tracked footprint + safety buffer |
| Costmap update | 15 Hz | Balance perception + compute |
| DWA samples | vx: 15, vtheta: 25 | Wide angular range for pivot capability |
| Reverse max speed | 0.8 m/s | Tracks reverse well (symmetric) |
| Track slip threshold | 15% | Encoder vs Point-LIO mismatch |

---

## 📦 ROS2 Packages

```
tank_ws/src/
├── tank_bringup          # Launch files & master configs
├── tank_localization     # Point-LIO + GNSS fusion
├── tank_perception       # LiDAR + camera → costmap
├── tank_navigation       # Global + local planning
├── tank_control          # ODrive interface + safety
├── tank_sensors          # Driver wrappers (L2, camera, GNSS)
├── tank_description      # URDF, extrinsics
├── tank_msgs             # Custom messages
├── tank_utils            # Logging, monitoring, tools
└── external/             # Submodules (Point-LIO, unilidar_sdk2, Patchwork++)
```

See [workspace_structure.md](workspace_structure.md) for full details.

---

## 🚀 Quick Start

### Fresh Jetson Installation

**One-command installation for new Jetson Orin Nano:**
```bash
cd ~/Tank_projects/freshstart
./00_install_all.sh
```
This installs all prerequisites, ODrive, ROS 2 Humble, sensor drivers, and configures the workspace. See [freshstart/README.md](freshstart/README.md) for details.

**Time:** 45-60 minutes

### Hardware Setup

#### 1. Dual Unitree L2 LiDARs
- Configure via serial: Front (192.168.2.62:6201), Rear (192.168.2.63:6202)
- See: [LIDAR_SETUP_COMPLETE.md](LIDAR_SETUP_COMPLETE.md)

#### 2. u-blox ZED-F9P GNSS
- Plug in via USB, then run: `python3 ~/Tank_projects/configure_gnss_ubx.py`
- See: [GNSS_SETUP_COMPLETE.md](GNSS_SETUP_COMPLETE.md)

#### 3. e-CAM25_CUONX Camera
- Build drivers on Linux PC using `ecam-25docs/build_ecam25_drivers.sh`
- See: [ECAM25_BUILD_OVERVIEW.md](ECAM25_BUILD_OVERVIEW.md)

#### 4. ODrive Motor Controller
- Plug in via USB, verify with: `odrivetool --version`
- See: [ODRIVE_JETSON_QUICKSTART.md](ODRIVE_JETSON_QUICKSTART.md)

### Launch System

```bash
# Source workspace
cd ~/Tank_projects/tank_ws
source install/setup.bash

# Launch dual LiDARs
ros2 launch tank_sensors dual_lidar.launch.py

# Launch GNSS (in another terminal)
ros2 launch tank_sensors gnss.launch.py

# Launch all sensors (when ready)
ros2 launch tank_sensors hardware.launch.py
```

### Test Hardware

```bash
# Test LiDAR network
~/Tank_projects/test_dual_lidar.sh

# Test GNSS
~/Tank_projects/test_gnss.sh

# Test ODrive
~/Tank_projects/test_odrive.sh
```

---

## 🧪 Field Test Protocol

### Phase 1: Core Localization (T1 - Parking Lot)
- Point-LIO + ODrive integration
- Encoder odometry validation
- Teleoperation @ 0.5-0.8 m/s
- **Pass criteria:** Drift <1% over 2-3 min, encoder matches Point-LIO within 15%

### Phase 2: LiDAR-Only Navigation (T2 - Moderate Terrain)
- Grass, shallow ruts, scattered rocks
- Patchwork++ ground extraction
- DWA planner with LiDAR-only costs
- **Pass criteria:** 100m autonomous run, 0 collisions, min clearance >0.3m

### Phase 3: Camera Fusion (T2 - Moderate Terrain)
- Train SegFormer-B0 on 500-1000 labeled frames from Phase 2
- Isaac ROS TensorRT inference
- Compare LiDAR-only vs camera-fused performance
- **Pass criteria:** 5-10% reduction in conservative stops, no false negatives

### Phase 4: Advanced Features (T3 - Rugged Terrain)
- GNSS waypoint navigation
- Reverse behaviors with rear LiDAR
- Recovery routines (stuck, belly strike)
- **Slope progression:** 15° → 20° → 25° → 30° (validate torque margin)

### Phase 5: MVP Validation
- **500m mixed terrain:** 0 collisions, ≤0.2 stuck events/km, 0 tip-overs
- **KPI tracking:** Pitch excursions, slip events, GNSS quality, motor temps
- **Parameter tuning:** Fine-tune terrain costs, speed schedule, inflation

---

## 📊 Key Performance Indicators (KPIs)

- **Safety:** Stuck events/km, collisions, tip-over events, pitch angle excursions
- **Efficiency:** % time in crawl/slow/fast, average speed
- **Robustness:** Localization drift without GNSS, LiDAR packet health %
- **Sensor health:** Segmentation confidence, GNSS fix quality, track slip %, motor temps

---

## ✅ Implementation Status

### Completed ✅
- [x] **Fresh installation system** - Automated scripts for Jetson setup
- [x] **Dual Unitree L2 LiDARs** - Network configuration, ROS 2 integration, dual launch files
- [x] **u-blox ZED-F9P GNSS** - UBX protocol configuration, 5 Hz position updates, cm-level accuracy with SBAS
- [x] **ODrive 0.5.4** - Installation, USB permissions, verification tools
- [x] **ROS 2 Humble** - Full installation with development tools
- [x] **Network setup** - LiDAR subnet (192.168.2.0/24), static IP configuration
- [x] **e-CAM25_CUONX** - Driver build scripts and documentation (ready for camera connection)

### In Progress 🔲
- [ ] **Point-LIO integration** - LiDAR-inertial odometry
- [ ] **Camera driver installation** - Requires camera connection and driver build
- [ ] **Navigation stack** - Path planning and control
- [ ] **Sensor fusion** - EKF with GNSS

### Pending 📋
- [ ] **Patchwork++** - Ground segmentation
- [ ] **Isaac ROS** - Camera semantic segmentation
- [ ] **Field testing** - Progressive test campaign (T1 → T2 → T3)

## ⚠️ Critical Validation Points

1. **ODrive firmware compatibility**
   - Currently using ODrive 0.5.4 (installed and verified)
   - If upgrading firmware, ensure compatibility with ROS 2 driver
   - See [ODRIVE_JETSON_QUICKSTART.md](ODRIVE_JETSON_QUICKSTART.md)

2. **30° slope target exceeds 15° manufacturer rating**
   - Test incrementally: 15° → 20° → 25° → 30°
   - Monitor ODrive motor current (<80% continuous rating)
   - At 30 lb payload: expect ~87% torque reserve

3. **GNSS accuracy and corridor width adaptation**
   - Current: 1-2cm accuracy with SBAS (excellent!)
   - Base corridor: 4m (good fix, HDOP <2.0)
   - Adapt to 5-7m based on DOP values (see GNSS_SETUP_COMPLETE.md)
   - GNSS loss: rely on Point-LIO for <500m missions

4. **Belly strike risk with 410mm contact length**
   - Monitor pitch oscillations + IMU Z-accel spikes
   - Recovery: back off 0.5m, re-approach at angle

5. **Track slip on 30° slopes**
   - 15% encoder vs Point-LIO threshold
   - Reduce speed to 0.5x if 15-30% slip detected
   - Stop if >30% slip (reassess traction/path)

---

## 🛠️ Development Tools

### Installation & Setup
- **Fresh installation scripts:** `freshstart/00_install_all.sh` - Complete automated setup
- **Hardware test scripts:** `test_dual_lidar.sh`, `test_gnss.sh`, `test_odrive.sh`
- **GNSS configuration:** `configure_gnss_ubx.py` - Automated UBX protocol setup

### Build Tools
- **Camera driver build:** `ecam-25docs/build_ecam25_drivers.sh` - Build on Linux PC
- **Camera driver install:** `ecam-25docs/install_ecam25_built.sh` - Install on Jetson

### Future Tools (Planned)
- **Rosbag recording:** Automated via `run_field_test.sh`
- **KPI dashboard:** Real-time monitoring during field tests
- **Parameter versioning:** Git tags for tested config sets
- **Model conversion:** ONNX → TensorRT FP16 scripts
- **Calibration tools:** Extrinsics, camera intrinsics helpers

---

## 📚 References

### Open Source Dependencies
- [Point-LIO ROS2](https://github.com/dfloreaa/point_lio_ros2) - LiDAR-inertial odometry
- [Unitree unilidar_sdk2](https://github.com/unitreerobotics/unilidar_sdk2) - L2 LiDAR driver
- [Patchwork++](https://github.com/url-kaist/patchwork-plusplus) - Ground segmentation
- [Isaac ROS](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_dnn_inference/index.html) - TensorRT inference

### Hardware
- [e-CAM25_CUONX](https://www.e-consystems.com/nvidia-cameras/jetson-orin-nx-cameras/full-hd-ar0234-global-shutter-camera.asp) - AR0234 global shutter camera
- [Unitree L2 LiDAR](https://www.unitree.com/) - 3D LiDAR with built-in IMU
- [ZED-F9P](https://www.u-blox.com/en/product/zed-f9p-module) - Multi-band GNSS

---

## 📝 License

[Choose: MIT, Apache 2.0, GPL, etc.]

---

## 🤝 Contributing

This is a personal project for a tracked autonomous rover. Contributions welcome after Phase 2 validation.

---

## 📞 Contact

[Your contact info]

---

---

## 📊 Current Status

**Phase:** Hardware Setup & Sensor Integration (Phase 0.5)

**Completed:**
- ✅ Fresh installation system for Jetson
- ✅ Dual Unitree L2 LiDARs configured and operational
- ✅ u-blox ZED-F9P GNSS configured with cm-level accuracy
- ✅ ODrive 0.5.4 installed and verified
- ✅ ROS 2 Humble fully configured
- ✅ Camera driver build scripts ready

**Next Steps:**
1. Connect and test e-CAM25_CUONX camera
2. Integrate Point-LIO for LiDAR-inertial odometry
3. Begin Phase 1: Core Localization & Control

**Documentation:** Comprehensive setup guides available for all hardware components. See project documents above.

---

**Last Updated:** November 4, 2025

