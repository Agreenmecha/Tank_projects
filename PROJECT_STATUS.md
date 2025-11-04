# Tank Autonomous Navigation - Project Status

**Last Updated:** 2025-01-31  
**Status:** Planning Phase Complete ✅

---

## 📋 **What's Been Completed**

### **1. Core Planning Documents**
| File | Description | Status |
|------|-------------|--------|
| `tank_plan.txt` | Master technical plan (hardware, algorithms, parameters) | ✅ Complete |
| `workspace_structure.md` | Full ROS2 workspace architecture & implementation guide | ✅ Complete |
| `README.md` | Project overview & quick start | ✅ Complete |
| `.gitignore` | Git exclusions for ROS2/Python/C++ | ✅ Complete |
| `QUICK_REFERENCE.md` | Field testing reference card (print this!) | ✅ Complete |

### **2. Test Scripts**
| File | Purpose | Status |
|------|---------|--------|
| `test_odrive_usb.py` | Python script for ODrive USB testing | ✅ Ready |

---

## 🎯 **Key Decisions Made**

### **Hardware Stack (Finalized)**
- ✅ **Platform:** Tracked vehicle (600mm width, 410mm contact, 229mm CG)
- ✅ **Compute:** Jetson Orin Nano (JetPack 6.2)
- ✅ **Sensors:**
  - 2x Unitree L2 3D LiDAR (front + rear)
  - e-CAM25_CUONX (AR0234 global shutter, 1280x720 @ 20fps)
  - ZED-F9P GNSS (no RTK - standard GNSS mode)
  - Wheel encoders (via ODrive)
- ✅ **Motor Control:** ODrive (firmware v0.5.6)
- ✅ **ODrive Interface:** USB (native protocol) - configured and tested

### **Software Stack (Finalized)**
- ✅ **Localization:** Point-LIO ROS2 (front L2 + built-in IMU)
- ✅ **GNSS Driver:** aussierobots/ublox_dgnss
- ✅ **Ground Extraction:** Patchwork++ (handles 30° slopes)
- ✅ **Camera Segmentation:** Isaac ROS + SegFormer-B0 (TensorRT)
- ✅ **Planning:** DWA local planner (tracked vehicle kinematics)
- ✅ **ODrive Interface:** USB (native protocol) - chosen and configured

### **Target Capabilities**
- ✅ Max speed: 1.5 m/s (conservative, 56% of 2.7 m/s rating)
- ✅ Max slope: 30° (11.8° safety margin to 41.8° tip angle)
- ✅ Corridor width: 4-5m (adaptive to GNSS quality)
- ✅ Reverse capable: 0.8 m/s using rear L2

---

## ⚠️ **Open Items (To Be Resolved)**

### **Critical (Before Phase 1)**
1. **ODrive Interface:** ✅ **DECIDED - USB**
   - [x] USB interface chosen (native protocol via odrivetool)
   - [x] ODrive 0.5.4 installed and verified
   - [x] USB udev rules configured
   - [ ] Implement ODrive ROS 2 node using USB interface

2. **Hardware Measurements:**
   - [x] ✅ Verify Unitree L2 specs (6-axis IMU, hardware time-synced)
   - [ ] Measure actual belly clearance
   - [ ] Verify IMU saturation limits (test empirically or contact Unitree)

### **Important (Before Phase 2-3)**
3. **Camera Training Data:**
   - [ ] Decide: collect during Phase 2 OR find existing dataset
   - [ ] Label 500-1000 frames for SegFormer-B0

4. **RC Override:**
   - [ ] Define if manual remote control backup is needed
   - [ ] Select hardware (e.g., RadioMaster TX16S + receiver)

### **Validation (Phase 4-5)**
5. **30° Slope Testing:**
   - [ ] Incremental validation: 15° → 20° → 25° → 30°
   - [ ] Monitor ODrive motor current (<80% continuous rating)
   - [ ] Validate torque reserve at 30 lb payload (87% available)

---

## 📦 **ROS2 Packages to Implement**

### **Phase 1: Core Localization (2-3 weeks)**
- [ ] `tank_sensors` - Sensor driver wrappers
- [ ] `tank_control` - ODrive interface (USB via odrivetool)
- [ ] `tank_localization` - Point-LIO + GNSS fusion
- [ ] `tank_description` - URDF with sensor transforms

### **Phase 2: LiDAR-Only Navigation (3-4 weeks)**
- [ ] `tank_perception` - Ground extraction + costmap
- [ ] `tank_navigation` - DWA planner + global planner

### **Phase 3: Camera Perception (2-3 weeks)**
- [ ] `tank_perception` - Add camera segmentation + fusion
- [ ] Isaac ROS integration

### **Phase 4: Advanced Features (2-3 weeks)**
- [ ] `tank_navigation` - Reverse mode + recovery behaviors
- [ ] Slope progression testing

### **Phase 5: Field Validation (1-2 weeks)**
- [ ] `tank_utils` - Logging, monitoring, KPI dashboard
- [ ] T1/T2/T3 field tests

**Total Estimated Time:** 10-15 weeks

---

## 🚀 **Next Steps (When Ready to Implement)**

### **Step 1: Set Up Repository (30 min)**
```bash
# On your development machine
cd /home/ubuntulaptop/Tank_projects

# Initialize git
git init
git add tank_plan.txt workspace_structure.md README.md .gitignore QUICK_REFERENCE.md
git commit -m "Initial planning phase - 30° tracked rover with CAN/GNSS"

# Create GitHub repo and push
gh repo create tank_autonomous_nav --public
git remote add origin git@github.com:YourUsername/tank_autonomous_nav.git
git push -u origin main
```

### **Step 2: ODrive USB Interface (✅ Completed)**
```bash
# On Jetson Orin Nano
# ODrive 0.5.4 installed via USB
odrivetool --version

# Test connection
odrivetool list
```

### **Step 3: Create ROS2 Workspace (1 day)**
```bash
# On Jetson or development machine
cd ~/tank_autonomous_nav
mkdir -p tank_ws/src

# Clone submodules
cd tank_ws/src
mkdir external
cd external
git clone https://github.com/dfloreaa/point_lio_ros2.git
git clone https://github.com/unitreerobotics/unilidar_sdk2.git
git clone https://github.com/url-kaist/patchwork-plusplus.git
git clone https://github.com/aussierobots/ublox_dgnss.git

# ODrive: USB interface (chosen)
# Using native odrivetool protocol via USB
# Custom ROS 2 node will wrap odrivetool for control

# Build
cd ~/tank_autonomous_nav/tank_ws
colcon build --symlink-install
```

### **Step 4: Begin Phase 1 Implementation**
See `workspace_structure.md` for detailed package structure and code examples.

---

## 📚 **Documentation Reference**

| Document | Use When |
|----------|----------|
| `tank_plan.txt` | Need parameters, hardware specs, or algorithm details |
| `workspace_structure.md` | Implementing ROS2 packages, need code examples |
| `README.md` | Showing project overview to others |
| `QUICK_REFERENCE.md` | In the field testing (print and laminate!) |

---

## 🔗 **Key External Resources**

### **Drivers & Libraries**
- [Point-LIO ROS2](https://github.com/dfloreaa/point_lio_ros2) - LiDAR-inertial odometry
- [Unitree unilidar_sdk2](https://github.com/unitreerobotics/unilidar_sdk2) - L2 LiDAR driver
- [aussierobots/ublox_dgnss](https://github.com/aussierobots/ublox_dgnss) - ZED-F9P GNSS driver
- [Patchwork++](https://github.com/url-kaist/patchwork-plusplus) - Ground segmentation
- [ODrive ROS2 Control](https://github.com/Factor-Robotics/odrive_ros2_control) - Motor control
- [Isaac ROS](https://nvidia-isaac-ros.github.io/) - TensorRT inference

### **Hardware Documentation**
- [ODrive v0.5.6 Docs](https://docs.odriverobotics.com/v/0.5.6/)
- [ODrive USB Protocol](https://docs.odriverobotics.com/v/0.5.6/api-reference.html)
- [e-CAM25_CUONX Camera](https://www.e-consystems.com/nvidia-cameras/jetson-orin-nx-cameras/full-hd-ar0234-global-shutter-camera.asp)

---

## 📊 **Project Timeline**

```
Phase 1: Core Localization          [████████░░░░░░░] 2-3 weeks
Phase 2: LiDAR-Only Navigation      [░░░░░░░░████░░░] 3-4 weeks
Phase 3: Camera Perception          [░░░░░░░░░░░███░] 2-3 weeks
Phase 4: Advanced Features          [░░░░░░░░░░░░███] 2-3 weeks
Phase 5: Field Validation           [░░░░░░░░░░░░░██] 1-2 weeks
                                     ─────────────────
                                     Total: 10-15 weeks
```

---

## ✅ **Success Criteria (MVP)**

After Phase 5, the system should achieve:
- ✅ 500m autonomous navigation on mixed terrain
- ✅ 0 collisions
- ✅ ≤0.2 stuck events per km
- ✅ 0 tip-over events
- ✅ Validated slope capability: ≥20° (stretch goal: 30°)
- ✅ Point-LIO drift: <1% without GNSS
- ✅ GNSS fix rate: >80%

---

**Status:** Ready to begin implementation whenever you have access to the Jetson! 🚀

