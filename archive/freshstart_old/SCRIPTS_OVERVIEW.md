# Fresh Installation Scripts Overview

Complete automated installation system for Tank Autonomous Navigation project on Jetson Orin Nano.

---

## Script Files

### Main Installation Script
**`00_install_all.sh`** - Master script that runs all installation steps in order
- Runs all numbered scripts sequentially
- Provides progress feedback
- Logs everything to `installation.log`
- Exits on first error for safety

### Individual Installation Scripts

**`01_install_system_prerequisites.sh`**
- Updates apt package lists
- Installs build tools (gcc, g++, cmake, make)
- Installs Python 3 and pip
- Installs USB and serial tools
- Installs network utilities
- Adds `~/.local/bin` to PATH

**`02_install_odrive.sh`**
- Installs ODrive 0.5.4 Python package
- Adds ODrive udev rules for USB access
- Adds user to dialout and plugdev groups
- Verifies odrivetool installation

**`03_install_ros2.sh`**
- Configures locale for UTF-8
- Adds ROS 2 repository and GPG key
- Installs ROS 2 Humble Desktop
- Installs ros-dev-tools
- Initializes rosdep
- Adds ROS 2 sourcing to ~/.bashrc

**`04_install_ros2_dev_tools.sh`**
- Installs colcon build system extensions
- Installs ROS 2 message packages (sensor_msgs, geometry_msgs, nav_msgs)
- Installs PCL and PCL-ROS for point cloud processing
- Installs RTCM messages for GNSS
- Installs robot_localization for sensor fusion
- Installs Navigation2 for path planning
- Installs TF2 for coordinate transforms
- Installs RViz2 and plugins for visualization
- Configures ROS_DOMAIN_ID and RMW_IMPLEMENTATION

**`05_install_sensor_drivers.sh`**
- Creates workspace directory structure
- Clones Unitree L2 LiDAR driver (unilidar_sdk2)
- Clones u-blox ZED-F9P GNSS driver (ublox_dgnss)
- Adds GNSS udev rules
- Builds both sensor drivers

**`06_setup_workspace.sh`**
- Creates tank_sensors ROS 2 package
- Creates launch and config directories
- Builds tank_sensors package
- Adds workspace sourcing to ~/.bashrc

**`07_configure_hardware.sh`**
- Configures LiDAR network (192.168.2.100/24 on eno1)
- Creates utility test scripts:
  - `test_dual_lidar.sh` - Tests LiDAR network connectivity
  - `test_gnss.sh` - Tests GNSS detection and node
  - `test_odrive.sh` - Tests ODrive installation
- Creates documentation and log directories

### Utility Scripts

**`99_cleanup.sh`** - Cleanup and reset script
- Removes ROS 2 packages
- Removes ODrive installation
- Cleans workspace build artifacts
- Removes configurations from ~/.bashrc
- Backs up ~/.bashrc before modifications

---

## Installation Flow

```
00_install_all.sh
    │
    ├─> 01_install_system_prerequisites.sh
    │       └─> Build tools, Python, USB/network tools
    │
    ├─> 02_install_odrive.sh
    │       └─> ODrive 0.5.4 + udev rules
    │
    ├─> 03_install_ros2.sh
    │       └─> ROS 2 Humble Desktop
    │
    ├─> 04_install_ros2_dev_tools.sh
    │       └─> PCL, RTCM, Nav2, TF2, RViz2, etc.
    │
    ├─> 05_install_sensor_drivers.sh
    │       └─> Unitree LiDAR + u-blox GNSS drivers
    │
    ├─> 06_setup_workspace.sh
    │       └─> tank_sensors package + workspace
    │
    └─> 07_configure_hardware.sh
            └─> Network setup + utility scripts
```

---

## File System Changes

### Installed Packages
```
/opt/ros/humble/                  # ROS 2 Humble installation
~/.local/bin/odrivetool           # ODrive command-line tool
~/.local/lib/python3.*/odrive/    # ODrive Python package
```

### Project Structure
```
~/Tank_projects/
├── freshstart/                   # Installation scripts (this directory)
├── tank_ws/                      # ROS 2 workspace
│   ├── src/
│   │   ├── tank_sensors/         # Custom sensors package
│   │   └── external/
│   │       ├── unilidar_sdk2/    # Unitree LiDAR driver
│   │       └── ublox_dgnss/      # u-blox GNSS driver
│   ├── build/
│   └── install/
├── test_dual_lidar.sh            # LiDAR test utility
├── test_gnss.sh                  # GNSS test utility
├── test_odrive.sh                # ODrive test utility
├── configure_gnss_ubx.py         # GNSS configuration script
├── docs/                         # Documentation
└── logs/                         # Log files
```

### System Configuration
```
/etc/udev/rules.d/
├── 91-odrive.rules               # ODrive USB permissions
└── 99-ublox.rules                # u-blox GNSS USB permissions

~/.bashrc additions:
├── export PATH=$PATH:~/.local/bin
├── source /opt/ros/humble/setup.bash
├── source ~/Tank_projects/tank_ws/install/setup.bash
├── export ROS_DOMAIN_ID=42
└── export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

### Network Configuration
```
NetworkManager connections:
└── lidar-network                 # Static IP 192.168.2.100/24 on eno1
```

---

## Dependencies Installed

### System Packages
```
build-essential, cmake, git, curl, wget
python3, python3-pip, python3-dev
libusb-1.0-0-dev, usbutils
tcpdump, net-tools, network-manager
```

### Python Packages
```
odrive==0.5.4
pyserial
```

### ROS 2 Packages
```
ros-humble-desktop
ros-humble-pcl-ros, ros-humble-pcl-conversions
ros-humble-rtcm-msgs
ros-humble-robot-localization
ros-humble-navigation2, ros-humble-nav2-bringup
ros-humble-tf2, ros-humble-tf2-ros
ros-humble-rviz2, ros-humble-rviz-default-plugins
ros-humble-rqt, ros-humble-rqt-common-plugins
```

### External Repositories
```
https://github.com/unitreerobotics/unilidar_sdk2.git
https://github.com/aussierobots/ublox_dgnss.git
```

---

## Error Handling

All scripts use `set -e` to exit on first error:
- Prevents cascading failures
- Allows safe retry after fixing issues
- Logs all output to `installation.log`

---

## Customization Points

Variables you can change before running:

### Network Configuration
```bash
# In 07_configure_hardware.sh
JETSON_IP=192.168.2.100
LIDAR_SUBNET=192.168.2.0/24
INTERFACE=eno1
```

### ROS 2 Configuration
```bash
# In 04_install_ros2_dev_tools.sh
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

### Workspace Location
```bash
# In multiple scripts
WORKSPACE_DIR=~/Tank_projects/tank_ws
```

---

## Testing

Each script can be tested individually:

```bash
# Test system prerequisites
./01_install_system_prerequisites.sh

# Verify
which gcc cmake git python3 pip3
```

```bash
# Test ODrive installation
./02_install_odrive.sh

# Verify
odrivetool --version
```

```bash
# Test ROS 2 installation
./03_install_ros2.sh

# Verify
ros2 --version
ros2 pkg list
```

---

## Recovery

If installation fails:

1. **Check the log:**
   ```bash
   tail -100 ~/Tank_projects/freshstart/installation.log
   ```

2. **Fix the issue** (install missing dependencies, fix network, etc.)

3. **Resume from failed step:**
   ```bash
   # Run only the failed script
   ./0X_scriptname.sh
   
   # Or start over
   ./99_cleanup.sh
   ./00_install_all.sh
   ```

---

## Documentation Files

- `README.md` - Complete installation guide
- `INSTALLATION_QUICKSTART.md` - Quick reference card
- `SCRIPTS_OVERVIEW.md` - This file (detailed script documentation)

---

## Version Information

- **Script Version:** 1.0
- **Target Platform:** Jetson Orin Nano
- **Target OS:** JetPack 6.2.1 (Ubuntu 22.04)
- **ROS 2 Version:** Humble
- **ODrive Version:** 0.5.4
- **Created:** November 4, 2025

---

## Notes

### Manual Steps Required After Installation

1. **LiDAR Configuration** (via serial):
   - Connect to each LiDAR via UART
   - Set IP addresses and target ports
   - Enable ENET mode

2. **GNSS Configuration:**
   - Run `configure_gnss_ubx.py` to enable UBX protocol
   - Verify with `test_gnss.sh`

3. **ODrive Calibration:**
   - Connect motors
   - Run calibration via `odrivetool`

### Assumptions

- Fresh JetPack 6.2.1 installation
- Internet connection available
- User has sudo privileges
- Wired Ethernet interface is `eno1`
- WiFi interface is available for field access

---

**All scripts are idempotent and safe to re-run if needed!**

