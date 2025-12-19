# Tank Projects Directory Guide

## 📁 Directory Structure

```
Tank_projects/
├── tank_ws/              # ROS2 workspace (primary development)
│   └── src/              # ROS2 packages
│       ├── master_bot/      # Robot URDF, description, Gazebo launch
│       ├── tank_control/    # Motor control (ODrive interface, teleop)
│       ├── tank_sensors/    # Sensor drivers (GNSS, LiDAR, camera)
│       ├── tank_msgs/       # Custom ROS messages
│       └── ...
│
├── scripts/              # Utility scripts
│   ├── rover              # SSH bridge to rover (v2)
│   ├── rover_ssh_bridge_v2.py
│   ├── odrive/           # ODrive testing scripts
│   └── archive/          # Old SSH v1 docs
│
├── docs/                 # Documentation
│   ├── guides/           # How-to guides
│   ├── reference/        # Technical reference
│   ├── setup/            # Setup instructions
│   ├── router/           # Router/network setup
│   └── remote-viz/       # Remote visualization guides
│
├── drivers/              # Hardware drivers (camera, etc)
├── ecam-25docs/          # e-CAM25 camera documentation
├── ecam25_drivers_deploy/# Camera driver deployment
├── odrive_docs/          # ODrive documentation
├── unitree_manual/       # Unitree LiDAR manual
│
├── archive/              # Completed phases and old docs
│   ├── PHASE1_IMPLEMENTATION.md
│   ├── freshstart_old/
│   └── ...
│
└── README.md             # Project overview
```

## 🔑 Key Files

### Configuration
- `tank_ws/src/tank_control/config/odrive_params.yaml` - Motor control config
- `tank_ws/src/tank_sensors/config/` - Sensor configurations

### Launch Files
- `tank_ws/src/master_bot/launch/gazebo.launch.py` - Gazebo simulation
- `tank_ws/src/tank_control/launch/odrive_interface.launch.py` - Motor control
- `tank_ws/src/tank_sensors/launch/hardware.launch.py` - All sensors

### Documentation
- `README.md` - Project overview
- `scripts/ROVER_SSH_README.md` - SSH bridge usage
- `scripts/ROVER_QUICK_REFERENCE.md` - Quick command reference

## 🚀 Quick Start

### Build Workspace
```bash
cd tank_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### Launch Robot
```bash
# Motor control
ros2 launch tank_control odrive_interface.launch.py

# Sensors
ros2 launch tank_sensors hardware.launch.py

# Simulation
ros2 launch master_bot gazebo.launch.py
```

### Remote Control
```bash
# From desktop, access rover
cd scripts
./rover exec "command"
./rover session-create my_session "ros2 launch ..."
```

## 📝 Notes

- `log/` - ROS2 logs (ignored by git)
- `archive/` - Completed phases, old documentation
- `.cursor/rules/` - Cursor AI project context
