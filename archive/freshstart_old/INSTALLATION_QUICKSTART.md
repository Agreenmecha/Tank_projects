# Fresh Installation Quick Start

## Prerequisites
- Jetson Orin Nano with JetPack 6.2.1
- Internet connection
- Sudo password ready

---

## Installation (One Command)

```bash
cd ~/Tank_projects/freshstart
./00_install_all.sh
```

**Time:** 45-60 minutes

---

## What Gets Installed

| Component | Version | Purpose |
|-----------|---------|---------|
| **System Tools** | Latest | Build tools, Python, USB/network utilities |
| **ODrive** | 0.5.4 | Motor controller software |
| **ROS 2** | Humble | Robot Operating System |
| **PCL/PCL-ROS** | Latest | Point cloud processing |
| **Unitree LiDAR** | Latest | L2 LiDAR driver |
| **u-blox GNSS** | Latest | ZED-F9P driver |
| **Navigation2** | Latest | Path planning (future) |

---

## Post-Installation Steps

### 1. Restart Terminal
```bash
source ~/.bashrc
```

### 2. Verify Installation
```bash
# Check ROS 2
ros2 --version

# Check ODrive
odrivetool --version

# Check workspace
source ~/Tank_projects/tank_ws/install/setup.bash
ros2 pkg list | grep tank_sensors
```

### 3. Configure Hardware

**LiDARs (via serial connection to each unit):**
```
Front LiDAR:
  IP: 192.168.2.62
  Target IP: 192.168.2.100
  Target Port: 6201
  Mode: ENET, Normal

Rear LiDAR:
  IP: 192.168.2.63
  Target IP: 192.168.2.100
  Target Port: 6202
  Mode: ENET, Normal
```

**GNSS (ZED-F9P):**
```bash
# Plug in via USB, then run:
python3 ~/Tank_projects/configure_gnss_ubx.py
```

### 4. Test Systems
```bash
# Test LiDAR network
~/Tank_projects/test_dual_lidar.sh

# Test GNSS
~/Tank_projects/test_gnss.sh

# Test ODrive
~/Tank_projects/test_odrive.sh
```

---

## Individual Script Usage

Run scripts individually if needed:

```bash
cd ~/Tank_projects/freshstart

# System prerequisites
./01_install_system_prerequisites.sh

# ODrive
./02_install_odrive.sh

# ROS 2
./03_install_ros2.sh

# ROS 2 dev tools
./04_install_ros2_dev_tools.sh

# Sensor drivers
./05_install_sensor_drivers.sh

# Workspace setup
./06_setup_workspace.sh

# Hardware config
./07_configure_hardware.sh
```

---

## Troubleshooting

### Installation Fails
```bash
# Check log
cat ~/Tank_projects/freshstart/installation.log

# Clean and retry
./99_cleanup.sh
./00_install_all.sh
```

### "Command not found" after installation
```bash
# Restart terminal or:
source ~/.bashrc
```

### ROS 2 packages not found
```bash
source /opt/ros/humble/setup.bash
source ~/Tank_projects/tank_ws/install/setup.bash
```

### Permission errors
```bash
# Make scripts executable
chmod +x ~/Tank_projects/freshstart/*.sh
```

---

## Manual Configuration Required

These steps require user interaction:

1. **LiDAR IP addresses** - Configure via serial terminal
2. **ODrive calibration** - Run via `odrivetool`
3. **GNSS UBX configuration** - Run Python script
4. **Hardware connections** - Physical setup

---

## Quick Reference

| Task | Command |
|------|---------|
| Launch LiDARs | `ros2 launch tank_sensors dual_lidar.launch.py` |
| Launch GNSS | `ros2 launch tank_sensors gnss.launch.py` |
| View topics | `ros2 topic list` |
| Test LiDAR | `~/Tank_projects/test_dual_lidar.sh` |
| Test GNSS | `~/Tank_projects/test_gnss.sh` |
| Configure GNSS | `python3 ~/Tank_projects/configure_gnss_ubx.py` |

---

## Support Documentation

- `~/Tank_projects/PROJECT_STATUS.md`
- `~/Tank_projects/LIDAR_SETUP_COMPLETE.md`
- `~/Tank_projects/GNSS_SETUP_COMPLETE.md`
- `~/Tank_projects/ODRIVE_JETSON_QUICKSTART.md`
- `~/Tank_projects/QUICK_REFERENCE.md`

---

**Ready to install? Run `./00_install_all.sh` and grab some coffee! ☕**

