# tankbot - SolidWorks URDF Export

URDF description package exported from SolidWorks URDF exporter.

## Installation

If you want to use the Gazebo launch file, install Gazebo dependencies:

```bash
sudo apt install -y \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-plugins \
    ros-humble-gazebo-ros-pkgs
```

## Usage

### View Robot in RViz

```bash
cd ~/Tank_projects/tank_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tankbot display.launch.py
```

This launches:
- `robot_state_publisher` - Publishes TF transforms from URDF
- `joint_state_publisher` - Publishes joint states (for continuous track joints)
- `rviz2` - 3D visualization

### Spawn in Gazebo

```bash
ros2 launch tankbot gazebo.launch.py
```

**Note:** Requires Gazebo to be installed (see Installation above).

## Package Structure

```
tankbot/
├── urdf/
│   └── tankbot.urdf      # Main URDF file from SolidWorks
├── meshes/                # STL mesh files for visualization
├── launch/
│   ├── display.launch.py  # RViz visualization
│   └── gazebo.launch.py   # Gazebo simulation
└── config/
    └── joint_names_tankbot.yaml
```

## URDF Structure

The URDF includes:
- `base_link` - Main robot base
- Left track links: `l_l1` through `l_l7` (continuous joints)
- Right track links: `l_r1` through `l_r7` (continuous joints)
- Additional links: `l_FL2`, `l_BL2`

All joints are continuous type for the tracks, allowing unlimited rotation.

