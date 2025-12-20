# Tank Rover - Autonomous Navigation Platform

**Platform:** Jetson Orin Nano  
**ROS2:** Humble  
**Control:** ODrive v3.6 (Firmware v0.5.6)  
**Sensors:** Dual LiDAR (L2), u-blox GNSS, e-CAM25  

---

## 🎯 Project Overview

Autonomous tank rover capable of:
- Outdoor navigation (roads, sidewalks)
- LiDAR-based obstacle avoidance
- GNSS localization
- Differential drive control
- Simulation and real-world deployment

---

## 📁 Repository Structure

```
Tank_projects/
├── README.md                    # This file
├── tank_ws/                     # ROS2 workspace
│   └── src/
│       ├── tank_control/        # Motor control (ODrive interface)
│       ├── tank_description/    # URDF robot model
│       ├── tank_navigation/     # Nav2 autonomous navigation
│       ├── tank_sensors/        # Sensor drivers (LiDAR, GNSS, camera)
│       ├── tank_localization/   # Sensor fusion
│       └── ...
│
├── docs/                        # Documentation
│   ├── setup/                   # Initial setup guides
│   ├── guides/                  # User guides and tutorials
│   └── reference/               # Technical reference
│
├── scripts/                     # Utility scripts
│   ├── odrive/                  # ODrive motor control scripts
│   ├── sensors/                 # Sensor testing scripts
│   └── setup_ros_network.sh     # ROS2 network configuration
│
├── drivers/                     # External drivers
├── odrive_docs/                 # ODrive documentation
├── ecam-25docs/                 # Camera documentation
└── archive/                     # Deprecated/completed documents

```

---

## 🚀 Quick Start

### 1. Prerequisites
- Jetson Orin Nano with JetPack 6.x
- ROS2 Humble installed
- ODrive v3.6 with motors connected
- LiDAR and GNSS sensors

### 2. Build ROS2 Workspace
```bash
cd tank_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 3. Test ODrive Communication
```bash
cd scripts/odrive
./quick_odrive_test.py
```

### 4. Test Motor Control
```bash
./test_motor_velocity.py
```

### 5. Launch ROS2 Control
```bash
./launch_odrive_ros2.sh
```

---

## 📚 Documentation

### Setup Guides
- **[ODrive Setup](docs/setup/ODRIVE_JETSON_QUICKSTART.md)** - Motor controller configuration
- **[GNSS Setup](docs/setup/GNSS_SETUP.md)** - u-blox GPS configuration
- **[LiDAR Setup](docs/setup/LIDAR_NETWORK_SETUP.md)** - Dual LiDAR network configuration
- **[Router Setup](docs/setup/ROUTER_QUICKSTART.md)** - Xiaomi WiFi router as access point ⭐ NEW
- **[Fresh Install](docs/setup/FRESH_INSTALL_COMPLETE.md)** - Complete system setup

### User Guides
- **[OnShape to URDF](docs/guides/ONSHAPE_TO_URDF_GUIDE.md)** - Robot model creation
- **[Simulation & Navigation](docs/guides/SIMULATION_AND_NAVIGATION_PLAN.md)** - Gazebo + Nav2 setup
- **[Remote Visualization](docs/guides/REMOTE_VISUALIZATION.md)** - RViz over network

### Technical Reference
- **[Motor Configuration](docs/reference/MOTOR_CONFIG_SUMMARY.md)** - ODrive parameters
- **[ODrive ROS2 Node](docs/reference/ODRIVE_ROS2_NODE_COMPLETE.md)** - ROS2 interface details
- **[Quick Reference](docs/reference/QUICK_REFERENCE.md)** - Common commands

---

## 🛠️ Hardware Configuration

### Motors
- **Controller:** ODrive v3.6 (Firmware v0.5.6)
- **Gearbox:** 13:1 ratio
- **Max Speed:** 6000 RPM (100 turns/s at motor)
- **Current Limit:** 30A
- **Velocity Ramp:** 10 turns/s² (configurable 3-15)
- **Encoders:** 2048 CPR, incremental

### Sensors
- **LiDAR:** 2x Unitree L2 (front and rear)
- **GNSS:** u-blox NEO-M9N
- **Camera:** e-CAM25 (future integration)
- **IMU:** (if applicable)

### Dimensions
- **Track Width:** 600mm (0.60m)
- **Wheel Radius:** 100mm (0.10m)
- **Drive Type:** Differential drive

---

## 🎮 Control Modes

### Manual Control
```bash
# Keyboard teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Autonomous Navigation
```bash
# Launch Nav2 stack
ros2 launch tank_navigation nav2_sim.launch.py
```

### Direct Motor Control (Python)
```python
import odrive
from odrive.enums import *

dev0 = odrive.find_any()
dev0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
dev0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
dev0.axis0.controller.input_vel = 10  # turns/s
dev0.axis1.controller.input_vel = 10
```

---

## 📊 ROS2 Topics

### Subscribed
| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/emergency_stop` | `std_msgs/Bool` | Emergency stop |

### Published
| Topic | Type | Description |
|-------|------|-------------|
| `/odrive/encoder_odom` | `nav_msgs/Odometry` | Wheel odometry |
| `/odrive/motor_status` | `sensor_msgs/JointState` | Motor status |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR data |
| `/fix` | `sensor_msgs/NavSatFix` | GNSS position |

### Services
| Service | Type | Description |
|---------|------|-------------|
| `/odrive/clear_errors` | `std_srvs/Trigger` | Clear ODrive errors |
| `/odrive/calibrate` | `std_srvs/Trigger` | Run motor calibration |

---

## 🔧 Useful Scripts

### ODrive Control
```bash
# Quick connection test
scripts/odrive/quick_odrive_test.py

# Full motor test
scripts/odrive/test_motor_velocity.py

# Emergency idle
scripts/odrive/idle_motors.py

# Set velocity ramp rate (3-15)
scripts/odrive/set_ramp_rate.py 10

# Launch ROS2 interface
scripts/odrive/launch_odrive_ros2.sh
```

### Sensor Testing
```bash
# Test GNSS
scripts/sensors/test_gnss.sh

# Test dual LiDAR
scripts/sensors/test_dual_lidar.sh
```

---

## 🐛 Troubleshooting

### ODrive Not Found
```bash
# Check USB connection
lsusb | grep ODrive

# Fix permissions (run once)
sudo scripts/odrive/fix_odrive_permissions.sh

# Then log out and back in, or:
newgrp dialout
```

### Motors Not Moving
```bash
# Check for errors
ros2 topic echo /odrive/errors

# Clear errors
ros2 service call /odrive/clear_errors std_srvs/srv/Trigger

# Test direct connection
scripts/odrive/quick_odrive_test.py
```

### LiDAR Connection Issues
```bash
# Check network
ping 192.168.2.62  # Front LiDAR
ping 192.168.2.63  # Rear LiDAR

# Reconfigure network
scripts/sensors/setup_lidar_network.sh
```

---

## 🔄 Development Workflow

### Simulation First
1. Create/import URDF model
2. Test in Gazebo simulation
3. Develop navigation algorithms
4. Tune parameters in sim

### Transfer to Real Robot
1. Same ROS2 nodes work on real hardware
2. Replace simulated sensors with real drivers
3. Same Nav2 stack controls both
4. Fine-tune parameters on real robot

---

## 📈 Project Status

### ✅ Completed
- [x] Jetson Orin Nano setup
- [x] ROS2 Humble installation
- [x] ODrive USB communication
- [x] Motor velocity control
- [x] ROS2 motor interface node
- [x] Dual LiDAR network setup
- [x] GNSS driver configuration
- [x] Workspace structure

### ⏳ In Progress
- [ ] Nav2 autonomous navigation
- [ ] GPS waypoint navigation
- [ ] Camera integration

### ✅ Recently Completed
- [x] Point-LIO sensor fusion
- [x] URDF robot model
- [x] Nav2 configuration

---

## 🤝 Contributing

This is a personal project, but feel free to:
- Report issues
- Suggest improvements
- Share similar projects

---

## 📝 License

[Add your license here]

---

## 📧 Contact

[Add your contact info here]

---

## 🔗 References

- **ROS2 Humble:** https://docs.ros.org/en/humble/
- **Nav2:** https://navigation.ros.org/
- **ODrive:** https://docs.odriverobotics.com/v/0.5.6/
- **Gazebo:** https://gazebosim.org/

---

**Last Updated:** December 19, 2025  
**Version:** 1.1
