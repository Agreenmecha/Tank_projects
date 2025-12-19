# Fresh Jetson Installation Scripts

**Purpose:** Automated installation of all prerequisites and dependencies for the Tank Autonomous Navigation project on a fresh Jetson Orin Nano with JetPack 6.2.1.

---

## Quick Start (Full Installation)

```bash
cd ~/Tank_projects/freshstart
chmod +x *.sh
./00_install_all.sh
```

**Total time:** ~45-60 minutes (depending on internet speed)

---

## Installation Order

If you want to run scripts individually:

```bash
# 1. System prerequisites and tools
./01_install_system_prerequisites.sh

# 2. ODrive motor controller
./02_install_odrive.sh

# 3. ROS 2 Humble
./03_install_ros2.sh

# 4. ROS 2 development tools
./04_install_ros2_dev_tools.sh

# 5. Sensor drivers (LiDAR, GNSS)
./05_install_sensor_drivers.sh

# 6. Project workspace setup
./06_setup_workspace.sh

# 7. Network and device configuration
./07_configure_hardware.sh
```

---

## What Gets Installed

### System Tools
- Build essentials (gcc, g++, make, cmake)
- Git, curl, wget
- Python 3.10+ with pip
- Network tools (tcpdump, net-tools, NetworkManager)
- USB tools (libusb)

### ODrive 0.5.4
- Python ODrive package
- odrivetool CLI
- USB udev rules

### ROS 2 Humble
- ROS 2 base installation
- ROS 2 development tools
- colcon build system
- ROS 2 dependencies:
  - pcl_conversions
  - pcl_ros
  - rtcm_msgs
  - robot_localization (planned)

### Sensor Drivers
- Unitree L2 LiDAR driver (unilidar_sdk2)
- u-blox ZED-F9P GNSS driver (ublox_dgnss)
- Python dependencies (pyserial)

### Project Structure
- tank_ws ROS 2 workspace
- tank_sensors package
- Dual LiDAR launch files
- GNSS configuration and launch files
- Network setup scripts
- Test and diagnostic scripts

---

## Requirements

### Hardware
- Jetson Orin Nano (tested on JetPack 6.2.1)
- Internet connection (for package downloads)
- At least 10GB free disk space

### Before Starting
- Fresh JetPack 6.2.1 installation
- Sudo password ready
- Internet connected (WiFi or Ethernet)

---

## Post-Installation

After installation completes, you'll need to:

1. **Connect hardware:**
   - Plug in dual Unitree L2 LiDARs via network switch
   - Connect ZED-F9P GNSS via USB
   - Connect ODrive motor controller via USB

2. **Configure LiDARs:**
   - Set Front LiDAR: IP 192.168.2.62, Target 192.168.2.100:6201
   - Set Rear LiDAR: IP 192.168.2.63, Target 192.168.2.100:6202
   - (Use Unitree configuration tool via serial)

3. **Configure GNSS:**
   - Run: `python3 ~/Tank_projects/configure_gnss_ubx.py`
   - Or use u-center 2 to enable UBX output

4. **Test systems:**
   ```bash
   # Test LiDAR network
   ~/Tank_projects/test_dual_lidar.sh
   
   # Test GNSS
   ~/Tank_projects/test_gnss.sh
   
   # Test ODrive
   odrivetool --version
   ```

---

## Troubleshooting

### Script Fails with Permission Error
```bash
chmod +x freshstart/*.sh
```

### "Package not found" Errors
- Check internet connection
- Run: `sudo apt update`
- Retry failed script

### ROS 2 Not in PATH
```bash
source /opt/ros/humble/setup.bash
# Or restart terminal
```

### ODrive Not Found
```bash
# Add to PATH manually
echo 'export PATH=$PATH:~/.local/bin' >> ~/.bashrc
source ~/.bashrc
```

---

## Manual Steps Not Covered

These require user interaction and can't be fully automated:

1. **LiDAR IP Configuration** (via serial connection)
2. **ODrive Motor Calibration** (via odrivetool)
3. **GNSS Antenna Placement** (physical setup)
4. **Network IP Assignment** (if not using 192.168.2.x)

---

## Recovery

If installation fails midway:

```bash
# Clean and restart
cd ~/Tank_projects/freshstart
./99_cleanup.sh
./00_install_all.sh
```

---

## Customization

Edit these variables in scripts if needed:

**Network Configuration:**
- `JETSON_IP=192.168.2.100`
- `LIDAR_SUBNET=192.168.2.0/24`

**ROS 2 Domain:**
- `ROS_DOMAIN_ID=42`

**Installation Paths:**
- `WORKSPACE=~/Tank_projects/tank_ws`

---

## Support

For issues, check:
- `~/Tank_projects/PROJECT_STATUS.md`
- `~/Tank_projects/LIDAR_SETUP_COMPLETE.md`
- `~/Tank_projects/GNSS_SETUP_COMPLETE.md`
- `~/Tank_projects/ODRIVE_JETSON_QUICKSTART.md`

---

**Version:** 1.0  
**Last Updated:** November 4, 2025  
**Tested On:** Jetson Orin Nano, JetPack 6.2.1

