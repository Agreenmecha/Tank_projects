# Tank Projects - Directory Structure

Clean, organized structure for GitHub

---

## 📁 Main Directories

```
Tank_projects/
├── README.md                    # Main project documentation
├── .gitignore                   # Git ignore rules
├── DIRECTORY_STRUCTURE.md       # This file
│
├── tank_ws/                     # ROS2 Workspace
│   ├── src/                     # Source packages
│   ├── build/                   # Build artifacts (gitignored)
│   ├── install/                 # Installed packages (gitignored)
│   └── log/                     # Build logs (gitignored)
│
├── docs/                        # Documentation
│   ├── setup/                   # Setup & installation guides
│   ├── guides/                  # User guides & tutorials
│   └── reference/               # Technical reference docs
│
├── scripts/                     # Utility scripts
│   ├── odrive/                  # Motor control scripts
│   ├── sensors/                 # Sensor testing scripts
│   └── setup_ros_network.sh     # Network configuration
│
├── archive/                     # Deprecated/completed documents
├── drivers/                     # External driver packages
├── odrive_docs/                 # ODrive documentation
├── ecam-25docs/                 # Camera documentation
└── unitree_manual/              # LiDAR manuals

```

---

## 📄 Documentation (`docs/`)

### Setup Guides (`docs/setup/`)
Initial system configuration and hardware setup

- `ODRIVE_JETSON_QUICKSTART.md` - ODrive motor controller setup
- `GNSS_SETUP.md` - u-blox GPS configuration
- `LIDAR_NETWORK_SETUP.md` - Dual LiDAR network setup
- `FIELD_WIRELESS_SETUP.md` - WiFi configuration for field use
- `FRESH_INSTALL_COMPLETE.md` - Complete system installation

### User Guides (`docs/guides/`)
How-to guides and tutorials

- `ONSHAPE_TO_URDF_GUIDE.md` - Robot model creation workflow
- `SIMULATION_AND_NAVIGATION_PLAN.md` - Gazebo + Nav2 setup
- `UCENTER_CONFIGURATION_GUIDE.md` - GNSS configuration tool
- `REMOTE_VISUALIZATION.md` - RViz over network

### Technical Reference (`docs/reference/`)
Technical specifications and API documentation

- `MOTOR_CONFIG_SUMMARY.md` - ODrive parameters & calculations
- `ODRIVE_CONTROL_READY.md` - Quick reference for motor control
- `ODRIVE_ROS2_NODE_COMPLETE.md` - ROS2 node documentation
- `QUICK_REFERENCE.md` - Common commands cheat sheet
- `URDF_SETUP_CHECKLIST.md` - URDF verification checklist

---

## 🔧 Scripts (`scripts/`)

### ODrive Scripts (`scripts/odrive/`)
Motor control and testing

| Script | Purpose |
|--------|---------|
| `quick_odrive_test.py` | Quick connection test (no movement) |
| `test_motor_velocity.py` | Full motor velocity test |
| `idle_motors.py` | Emergency stop & idle motors |
| `set_ramp_rate.py` | Configure acceleration (3-15 turns/s²) |
| `set_velocity_limit.py` | Set max velocity (100 turns/s) |
| `launch_odrive_ros2.sh` | Launch ROS2 motor interface |
| `test_odrive_ros2.sh` | Test ROS2 node functionality |
| `test_odrive_usb.py` | Comprehensive USB test suite |
| `test_ros2_velocity.sh` | Test ROS2 velocity commands |
| `check_odrive_version.sh` | Verify ODrive firmware version |
| `fix_odrive_permissions.sh` | Fix USB permissions (run once) |

### Sensor Scripts (`scripts/sensors/`)
Sensor testing and configuration

| Script | Purpose |
|--------|---------|
| `configure_gnss_ubx.py` | Configure u-blox GNSS settings |
| `test_gnss.sh` | Test GNSS connection & data |
| `test_dual_lidar.sh` | Test both LiDAR sensors |
| `setup_lidar_network.sh` | Configure LiDAR network interfaces |

### General Scripts (`scripts/`)
System-wide utilities

| Script | Purpose |
|--------|---------|
| `setup_ros_network.sh` | Configure ROS2 network for multi-machine |

---

## 🤖 ROS2 Workspace (`tank_ws/src/`)

### Core Packages

| Package | Purpose |
|---------|---------|
| `tank_control` | Motor control (ODrive ROS2 interface) |
| `tank_description` | URDF robot model & visualization |
| `tank_navigation` | Nav2 autonomous navigation stack |
| `tank_sensors` | Sensor drivers (LiDAR, GNSS, camera) |
| `tank_localization` | Sensor fusion & odometry |
| `tank_bringup` | Launch files for full system |
| `tank_msgs` | Custom ROS2 message definitions |
| `tank_perception` | Obstacle detection & tracking |
| `tank_utils` | Utility nodes & tools |

### External Packages

| Package | Purpose |
|---------|---------|
| `external/` | Third-party ROS2 packages |

---

## 📦 Archive (`archive/`)

Deprecated or completed documentation (kept for reference)

- `GNSS_SETUP_COMPLETE.md` - Completed setup log
- `LIDAR_SETUP_COMPLETE.md` - Completed setup log
- `WORKSPACE_CREATED.md` - Initial workspace creation
- `MOTOR_CONTROL_IMPLEMENTATION.md` - Implementation notes
- `PHASE1_IMPLEMENTATION.md` - Phase 1 completion notes
- `PROJECT_STATUS.md` - Old status tracking
- `ECAM25_BUILD_OVERVIEW.md` - Camera build notes
- `workspace_structure.md` - Old structure doc
- `tank_plan.txt` - Initial planning notes

---

## 🚫 Gitignored Items

The following are excluded from version control:

### Build Artifacts
- `tank_ws/build/` - Compiled code
- `tank_ws/install/` - Installed packages
- `tank_ws/log/` - Build logs

### Python
- `__pycache__/` - Python bytecode
- `*.pyc`, `*.pyo` - Compiled Python

### IDE
- `.vscode/`, `.idea/` - IDE settings
- `*.swp`, `*.swo` - Vim swap files

### Logs
- `*.log` - Log files
- `log/` - Log directories

---

## 📋 File Naming Conventions

### Documentation
- `UPPERCASE_WITH_UNDERSCORES.md` - Major documentation
- Clear, descriptive names
- Use `.md` for markdown

### Scripts
- `snake_case.py` - Python scripts
- `kebab-case.sh` - Shell scripts
- Descriptive names indicating purpose

### ROS2 Packages
- `snake_case` - Package names
- Prefix with `tank_` for project packages

---

## 🔍 Finding Files

### Quick Reference
```bash
# Find all documentation
find docs/ -name "*.md"

# Find all ODrive scripts
ls scripts/odrive/

# Find all ROS2 packages
ls tank_ws/src/

# Find setup guides
ls docs/setup/
```

### Common Locations

| What | Where |
|------|-------|
| Main README | `README.md` |
| ODrive setup | `docs/setup/ODRIVE_JETSON_QUICKSTART.md` |
| Motor config | `docs/reference/MOTOR_CONFIG_SUMMARY.md` |
| Quick test | `scripts/odrive/quick_odrive_test.py` |
| Launch ROS2 | `scripts/odrive/launch_odrive_ros2.sh` |
| Nav2 config | `tank_ws/src/tank_navigation/config/` |
| URDF model | `tank_ws/src/tank_description/urdf/` |

---

## 🎯 Clean Structure Benefits

✅ **Easy Navigation** - Logical organization  
✅ **GitHub Ready** - Professional structure  
✅ **Clear Purpose** - Each directory has specific role  
✅ **Maintainable** - Easy to find and update files  
✅ **Scalable** - Room to grow  
✅ **Documented** - This file explains everything  

---

**Last Updated:** December 4, 2025

