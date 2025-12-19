---
description: "Tank project organization, key directories, and file locations"
alwaysApply: false
---

# Tank Project Structure

## Overview

This is a ROS 2 autonomous tank rover project with:
- Dual Unitree L2 LiDARs (front/rear)
- ZED-F9P GNSS (RTK-capable)
- e-CAM25_CUONX camera (AR0234 global shutter)
- ODrive motor controllers
- Jetson Orin Nano compute (rover)
- Ubuntu Desktop (development machine)

## Key Directories

```
Tank_projects/
├── tank_ws/                    # ROS 2 workspace
│   ├── src/                   # Source packages
│   │   ├── tank_sensors/      # Sensor drivers & launch files
│   │   ├── tank_bringup/      # System launch files
│   │   ├── tank_description/  # URDF robot model
│   │   ├── tank_localization/ # Point-LIO localization
│   │   ├── tank_control/      # Motor control
│   │   ├── master_bot/        # Base robot description
│   │   └── external/          # Third-party packages
│   ├── build/                 # Build artifacts (gitignored)
│   ├── install/               # Install space (gitignored)
│   └── log/                   # Logs (gitignored)
│
├── scripts/                    # Helper scripts
│   ├── rover                  # Main rover SSH command
│   ├── rover_ssh_bridge_v2.py # SSH bridge implementation
│   ├── rover_config.json      # SSH config (gitignored)
│   ├── odrive/               # ODrive test scripts
│   ├── sensors/              # Sensor test scripts
│   └── network/              # Network setup scripts
│
├── docs/                       # Documentation
│   ├── guides/               # How-to guides
│   ├── reference/            # Reference docs
│   └── setup/                # Setup instructions
│
├── ecam-25docs/               # Camera documentation & drivers
├── drivers/                   # Hardware drivers
├── freshstart/                # Clean install scripts
└── archive/                   # Old documentation
```

## Important Files

### Configuration
- `.gitignore` - Excludes build/, install/, log/, rover_config.json
- `scripts/rover_config.json` - Rover SSH credentials (not in git)

### Documentation
- `README.md` - Project overview
- `DIRECTORY_STRUCTURE.md` - Detailed project layout
- `scripts/ROVER_SSH_README.md` - Complete rover access guide
- `scripts/ROVER_QUICK_REFERENCE.md` - Quick command reference

### Launch Files
- `tank_ws/src/tank_sensors/launch/hardware.launch.py` - All sensors
- `tank_ws/src/tank_sensors/launch/lidar_dual.launch.py` - Dual LiDARs
- `tank_ws/src/tank_sensors/launch/gnss.launch.py` - GNSS only

## Hardware Details

### LiDARs
- **Type:** Unitree L2 (2x)
- **Position:** Front and rear mounted
- **Topics:** `/lidar_front/cloud`, `/lidar_rear/cloud`
- **Rate:** ~10 Hz
- **Connection:** Serial over USB

### GNSS
- **Type:** u-blox ZED-F9P
- **Capability:** RTK-enabled (dual-frequency)
- **Topics:** `/ubx_nav_pvt`, `/ubx_nav_hp_pos_llh`
- **Connection:** USB

### Camera
- **Type:** e-CAM25_CUONX (AR0234 sensor)
- **Features:** Global shutter, MIPI CSI-2
- **Driver:** e-con Systems custom (see ecam-25docs/)
- **Status:** Available but typically disabled

### Motor Control
- **Type:** ODrive S1 (quantity TBD)
- **Interface:** CAN bus
- **Scripts:** `scripts/odrive/`

## Network Configuration

- **ROS_DOMAIN_ID:** 42
- **Rover IP:** 192.168.2.100
- **Rover Hostname:** tankviaxiaomi
- **Desktop:** Dynamic IP on same network

## Git Workflow

- **Branch:** phase1-complete
- **Remote:** github.com/Agreenmecha/Tank_projects.git
- **Desktop → GitHub:** SSH authentication
- **Rover ← GitHub:** HTTPS or SSH

## Build System

- **ROS 2 Version:** Humble
- **Build Tool:** colcon
- **Build Type:** ament_cmake (most packages)
- **Symlink Install:** Enabled for fast Python iteration

## Common Patterns

### Creating New Packages
New packages go in `tank_ws/src/` with appropriate namespace:
- `tank_*` prefix for project packages
- Group by function: sensors, control, navigation, etc.

### Adding Launch Files
Launch files go in `<package>/launch/` directory.
Ensure `launch/` directory exists to avoid build errors.

### Documentation
- User guides → `docs/guides/`
- API reference → `docs/reference/`
- Setup instructions → `docs/setup/`

## Debugging Locations

- **Sensor logs:** Via `./rover session-output sensors`
- **Build logs:** `tank_ws/log/latest_build/`
- **Runtime logs:** `tank_ws/log/latest/`
- **System logs (rover):** `/var/log/syslog`

@DIRECTORY_STRUCTURE.md
@README.md

