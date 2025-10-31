# Tank Autonomous Navigation System

Autonomous off-road navigation system for a tracked platform using dual 3D LiDARs, camera semantic segmentation, and Point-LIO localization.

**Target Capability:** 30° slope navigation through rugged terrain with obstacle avoidance and confident reversing.

---

## 📋 Project Documents

| Document | Description |
|----------|-------------|
| **[tank_plan.txt](tank_plan.txt)** | Master technical plan - hardware specs, algorithms, parameters, KPIs |
| **[workspace_structure.md](workspace_structure.md)** | ROS2 workspace architecture, package organization, build scripts |

---

## 🛠️ Hardware Stack

### Platform
- **Tracked vehicle** (differential drive)
  - Track width: 600mm, Contact length: 410mm
  - CG height: 229mm (low center of gravity)
  - Rated: 100kg @ 6mph @ 15° slope
  - Typical payload: 30 lb (87% torque reserve)

### Compute
- **NVIDIA Jetson Orin Nano** (JetPack 6.2, L4T 36.4.3)
  - 68-73% CPU utilization (with Isaac ROS optimization)
  - 45-50% GPU utilization

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
  
- **ZED-F9P GNSS** (no RTK - standard GNSS)
  - ~2-5m accuracy
  - Multi-band (L1/L2) for improved performance

### Actuators
- **ODrive motor controller**
  - Velocity control mode
  - Watchdog timeout: 200ms
  - Wheel encoders for slip detection

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

### 1. Clone Repository
```bash
git clone --recursive https://github.com/YourUsername/tank_autonomous_nav.git
cd tank_autonomous_nav
```

### 2. Setup Workspace
```bash
./scripts/setup_workspace.sh
# Installs ROS2 Humble, builds packages, sets up Isaac ROS Docker
```

### 3. Launch System
```bash
# Full autonomous navigation
source tank_ws/install/setup.bash
ros2 launch tank_bringup tank_full.launch.py

# Or teleoperate for testing
ros2 launch tank_bringup teleop.launch.py
```

### 4. Run Autonomous Mission
```bash
ros2 run tank_navigation waypoint_commander \
  --waypoints "[[10,0], [10,10], [0,10]]"
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

## ⚠️ Critical Validation Points

1. **ODrive firmware compatibility issue**
   - Tank uses ODrive firmware v0.5.6
   - Official ROS2 Humble support: v0.5.3 and v0.5.1 only
   - **Action required:** Downgrade to v0.5.3 OR test v0.5.3 branch with v0.5.6 OR fork/adapt code
   - See [workspace_structure.md - ODrive Integration Notes](workspace_structure.md#odrive-integration-notes)

2. **30° slope target exceeds 15° manufacturer rating**
   - Test incrementally: 15° → 20° → 25° → 30°
   - Monitor ODrive motor current (<80% continuous rating)
   - At 30 lb payload: expect ~87% torque reserve

3. **No RTK - corridor width adaptation required**
   - Base corridor: 4-5m (GNSS ~2-5m accuracy)
   - Poor fix (HDOP >3, <6 satellites): widen to 6-7m
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

**Status:** Planning phase complete. Ready for Phase 1 implementation (Core Localization & Control).

