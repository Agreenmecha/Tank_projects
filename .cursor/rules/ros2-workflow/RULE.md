---
description: "ROS 2 development patterns, workspace structure, and build procedures"
globs: ["tank_ws/**/*"]
alwaysApply: false
---

# ROS 2 Workflow

## Workspace Structure

```
tank_ws/
├── src/
│   ├── tank_sensors/          # Sensor drivers (LiDAR, GNSS, Camera)
│   ├── tank_bringup/          # Launch files
│   ├── tank_description/      # URDF/robot model
│   ├── tank_localization/     # Point-LIO
│   ├── tank_control/          # Motor control
│   ├── master_bot/            # Base robot description
│   └── external/              # Third-party packages
├── build/                     # Build artifacts (gitignored)
├── install/                   # Install space (gitignored)
└── log/                       # Build/runtime logs (gitignored)
```

## Build Commands

### On Desktop
```bash
cd /home/aaron/Tank_projects/tank_ws
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42

# Full build
colcon build --symlink-install

# Specific package
colcon build --packages-select <package_name>

# Source after build
source install/setup.bash
```

### On Rover (via SSH)
```bash
# Quick rebuild
./rover exec "cd ~/Tank_projects/tank_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install" --timeout 180

# Or use session for long builds
./rover session-create build --command "cd ~/Tank_projects/tank_ws && colcon build --symlink-install"
./rover session-output build
```

## Launch Patterns

### Sensors
```bash
# All hardware sensors (LiDARs + GNSS)
ros2 launch tank_sensors hardware.launch.py

# With arguments
ros2 launch tank_sensors hardware.launch.py \
  enable_gnss:=true \
  enable_lidars:=true \
  enable_camera:=false
```

### On Rover
```bash
./rover ros-launch sensors tank_sensors hardware.launch.py
```

## Common ROS 2 Commands

```bash
# Topics
ros2 topic list
ros2 topic echo /topic_name
ros2 topic hz /topic_name
ros2 topic info /topic_name

# Nodes
ros2 node list
ros2 node info /node_name

# Parameters
ros2 param list
ros2 param get /node_name param_name

# TF
ros2 run tf2_ros tf2_echo parent_frame child_frame
ros2 run tf2_tools view_frames
```

## Key Topics

- `/lidar_front/cloud` - Front LiDAR point cloud (~10Hz)
- `/lidar_rear/cloud` - Rear LiDAR point cloud (~10Hz)
- `/lidar_front/imu` - Front LiDAR IMU
- `/ubx_nav_pvt` - GNSS position/velocity
- `/tf` - Transform tree
- `/tf_static` - Static transforms

## Package Dependencies

If a package fails to build, check for missing dependencies:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Creating New Packages

```bash
cd ~/Tank_projects/tank_ws/src
ros2 pkg create --build-type ament_cmake <package_name>
# or for Python
ros2 pkg create --build-type ament_python <package_name>
```

## Development Cycle

1. Edit code on desktop in Cursor
2. Commit and push: `git push`
3. Pull on rover: `./rover exec "cd ~/Tank_projects && git pull"`
4. Rebuild: `./rover session-create rebuild --command "cd ~/Tank_projects/tank_ws && colcon build --packages-select <package>"`
5. Restart nodes: `./rover session-kill sensors && ./rover ros-launch sensors ...`

## Important Notes

- Always source `/opt/ros/humble/setup.bash` before building
- Always source `install/setup.bash` after building
- Set `ROS_DOMAIN_ID=42` for network communication
- Use `--symlink-install` for faster Python package development
- Missing `launch/` or `config/` directories cause build errors - create them if needed

@tank_ws/src/tank_sensors/launch/hardware.launch.py

