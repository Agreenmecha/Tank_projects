# Fresh Installation System - Complete ✅

**Created:** November 4, 2025  
**Location:** `~/Tank_projects/freshstart/`

---

## What Was Created

A complete automated installation system for setting up the Tank Autonomous Navigation project on a fresh Jetson Orin Nano with JetPack 6.2.1.

### Installation Scripts (9 total)

| Script | Purpose | Time |
|--------|---------|------|
| `00_install_all.sh` | Master script - runs all others | 45-60 min |
| `01_install_system_prerequisites.sh` | System tools & Python | 5 min |
| `02_install_odrive.sh` | ODrive 0.5.4 motor controller | 2 min |
| `03_install_ros2.sh` | ROS 2 Humble installation | 15 min |
| `04_install_ros2_dev_tools.sh` | ROS 2 packages & dependencies | 10 min |
| `05_install_sensor_drivers.sh` | LiDAR & GNSS drivers | 10 min |
| `06_setup_workspace.sh` | tank_ws workspace setup | 3 min |
| `07_configure_hardware.sh` | Network & utility scripts | 2 min |
| `99_cleanup.sh` | Recovery/reset script | 2 min |

### Documentation (3 files)

| Document | Purpose |
|----------|---------|
| `README.md` | Complete installation guide with troubleshooting |
| `INSTALLATION_QUICKSTART.md` | Quick reference card for fast setup |
| `SCRIPTS_OVERVIEW.md` | Detailed technical documentation of all scripts |

---

## Complete Installation Coverage

### System Software
✅ Build tools (gcc, g++, cmake, make)  
✅ Python 3.10+ with pip  
✅ Git, curl, wget  
✅ USB and serial tools  
✅ Network utilities (tcpdump, NetworkManager)

### Motor Control
✅ ODrive 0.5.4 Python package  
✅ odrivetool CLI  
✅ USB udev rules for ODrive

### ROS 2 Ecosystem
✅ ROS 2 Humble Desktop  
✅ colcon build system  
✅ PCL and PCL-ROS (point clouds)  
✅ robot_localization (sensor fusion)  
✅ Navigation2 (path planning)  
✅ TF2 (coordinate transforms)  
✅ RViz2 with plugins  
✅ ROS 2 development tools

### Sensor Drivers
✅ Unitree L2 LiDAR driver (unilidar_sdk2)  
✅ u-blox ZED-F9P GNSS driver (ublox_dgnss)  
✅ RTCM messages for GNSS corrections

### Project Setup
✅ tank_ws ROS 2 workspace  
✅ tank_sensors package structure  
✅ Network configuration (192.168.2.0/24)  
✅ Utility test scripts

---

## Usage

### Quick Start (One Command)

```bash
cd ~/Tank_projects/freshstart
./00_install_all.sh
```

**That's it!** Script runs for 45-60 minutes and installs everything.

### Individual Component Installation

If you prefer step-by-step or need to fix a failed installation:

```bash
cd ~/Tank_projects/freshstart

# Run scripts in order
./01_install_system_prerequisites.sh
./02_install_odrive.sh
./03_install_ros2.sh
./04_install_ros2_dev_tools.sh
./05_install_sensor_drivers.sh
./06_setup_workspace.sh
./07_configure_hardware.sh
```

### Recovery/Cleanup

If something goes wrong:

```bash
cd ~/Tank_projects/freshstart
./99_cleanup.sh          # Remove everything
./00_install_all.sh      # Start fresh
```

---

## Post-Installation Steps

After scripts complete, you'll need to:

### 1. Restart Terminal
```bash
source ~/.bashrc
```

### 2. Configure LiDARs (via serial)

**Front LiDAR:**
- IP: 192.168.2.62
- Target: 192.168.2.100:6201
- Mode: ENET, Normal

**Rear LiDAR:**
- IP: 192.168.2.63
- Target: 192.168.2.100:6202
- Mode: ENET, Normal

### 3. Configure GNSS
```bash
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

## What Gets Configured

### Network
- Wired Ethernet (`eno1`): 192.168.2.100/24
- LiDAR subnet: 192.168.2.0/24
- Front LiDAR: 192.168.2.62
- Rear LiDAR: 192.168.2.63

### ROS 2 Environment
- ROS_DOMAIN_ID: 42
- RMW_IMPLEMENTATION: rmw_fastrtps_cpp
- Auto-sourcing in ~/.bashrc

### USB Permissions
- ODrive: /dev/bus/usb/* (via udev rules)
- GNSS: /dev/ttyACM0 (via udev rules)

---

## File Locations

### Installation Scripts
```
~/Tank_projects/freshstart/
├── 00_install_all.sh
├── 01_install_system_prerequisites.sh
├── 02_install_odrive.sh
├── 03_install_ros2.sh
├── 04_install_ros2_dev_tools.sh
├── 05_install_sensor_drivers.sh
├── 06_setup_workspace.sh
├── 07_configure_hardware.sh
├── 99_cleanup.sh
├── README.md
├── INSTALLATION_QUICKSTART.md
└── SCRIPTS_OVERVIEW.md
```

### Project Workspace
```
~/Tank_projects/tank_ws/
├── src/
│   ├── tank_sensors/          # Custom package
│   └── external/
│       ├── unilidar_sdk2/     # LiDAR driver
│       └── ublox_dgnss/       # GNSS driver
├── build/
└── install/
```

### Test Utilities
```
~/Tank_projects/
├── test_dual_lidar.sh
├── test_gnss.sh
├── test_odrive.sh
└── configure_gnss_ubx.py
```

---

## Features

### Robust Error Handling
- Scripts exit on first error
- All output logged to `installation.log`
- Safe to re-run scripts
- Cleanup script for recovery

### Idempotent Design
- Scripts check for existing installations
- Skip already-completed steps
- Won't break if run multiple times

### Modular Architecture
- Each script is independent
- Can run individually or all at once
- Easy to customize variables

### Comprehensive Logging
- All output captured
- Easy troubleshooting
- Installation history preserved

---

## System Requirements

### Hardware
- Jetson Orin Nano
- At least 10GB free disk space
- Internet connection

### Software
- JetPack 6.2.1 (Ubuntu 22.04)
- Sudo privileges
- Default shell: bash

---

## Verification

After installation, verify everything works:

### Check Installations
```bash
# ROS 2
ros2 --version
ros2 pkg list | grep tank

# ODrive
odrivetool --version

# Python packages
pip3 list | grep -E "odrive|pyserial"

# Workspace
source ~/Tank_projects/tank_ws/install/setup.bash
ros2 pkg list | grep tank_sensors
```

### Check Services
```bash
# Network interface
ip addr show eno1 | grep 192.168.2.100

# USB devices
lsusb

# Serial devices
ls -l /dev/ttyACM*
```

### Launch Test
```bash
# Source workspace
source ~/Tank_projects/tank_ws/install/setup.bash

# Test ROS 2
ros2 topic list

# Test node creation
ros2 run tank_sensors --help
```

---

## Troubleshooting

### Scripts Fail with Permission Error
```bash
chmod +x ~/Tank_projects/freshstart/*.sh
```

### ROS 2 Not Found After Installation
```bash
source ~/.bashrc
# or restart terminal
```

### Network Configuration Fails
```bash
# Check interface name
ip link show

# Modify in 07_configure_hardware.sh if not eno1
```

### ODrive Not in PATH
```bash
export PATH=$PATH:~/.local/bin
source ~/.bashrc
```

---

## Support Documentation

Additional documentation in main project:
- `PROJECT_STATUS.md` - Project overview and status
- `LIDAR_SETUP_COMPLETE.md` - LiDAR configuration guide
- `GNSS_SETUP_COMPLETE.md` - GNSS setup and usage
- `ODRIVE_JETSON_QUICKSTART.md` - ODrive quickstart
- `QUICK_REFERENCE.md` - Field operations guide
- `REMOTE_VISUALIZATION.md` - RViz remote setup
- `FIELD_WIRELESS_SETUP.md` - Wireless AP setup

---

## Success Criteria

Installation is complete when:

✅ All scripts run without errors  
✅ `ros2 --version` works  
✅ `odrivetool --version` works  
✅ `source ~/Tank_projects/tank_ws/install/setup.bash` works  
✅ Network interface configured (192.168.2.100)  
✅ Test scripts created  
✅ Documentation accessible

---

## Next Steps After Installation

1. **Configure LiDARs** - Connect via serial and set IPs
2. **Configure GNSS** - Run Python script to enable UBX
3. **Test Hardware** - Use test scripts
4. **Calibrate ODrive** - Connect motors and calibrate
5. **Field Test** - Verify all systems work together

---

## Version History

**v1.0** - November 4, 2025
- Initial release
- Complete automated installation
- Supports JetPack 6.2.1
- ROS 2 Humble
- ODrive 0.5.4
- Dual Unitree L2 LiDARs
- u-blox ZED-F9P GNSS

---

## Credits

Created for Tank Autonomous Navigation Project on Jetson Orin Nano.

**Tested on:**
- Hardware: Jetson Orin Nano 8GB
- OS: JetPack 6.2.1 (Ubuntu 22.04)
- ROS 2: Humble
- Date: November 4, 2025

---

**Ready to install on a fresh Jetson? Just run `./00_install_all.sh`!** 🚀

---

## Quick Command Reference

```bash
# Full installation
cd ~/Tank_projects/freshstart && ./00_install_all.sh

# Check log
tail -f ~/Tank_projects/freshstart/installation.log

# Verify installation
source ~/.bashrc && ros2 --version && odrivetool --version

# Test hardware
~/Tank_projects/test_dual_lidar.sh
~/Tank_projects/test_gnss.sh
~/Tank_projects/test_odrive.sh

# Clean and reinstall
cd ~/Tank_projects/freshstart
./99_cleanup.sh
./00_install_all.sh
```

---

**Everything you need for a fresh Jetson setup is ready to go!** ✅

