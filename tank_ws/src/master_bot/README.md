# master_bot - ROS 2 URDF Package

This package contains the URDF description for the master_bot robot, converted from ROS 1 to ROS 2.

## Package Contents

- **urdf/**: Robot URDF files
- **meshes/**: 3D mesh files (STL format)
- **launch/**: ROS 2 Python launch files
- **config/**: Configuration files including RViz config
- **textures/**: Texture files for visualization

## Building the Package

```bash
cd ~/Tank_projects/tank_ws
colcon build --packages-select master_bot
source install/setup.bash
```

## Usage

### Visualize in RViz

Launch the robot model in RViz with joint state publisher GUI:

```bash
ros2 launch master_bot display.launch.py
```

This will:
- Load the robot description
- Start the robot state publisher
- Open RViz2 for visualization
- Launch joint state publisher GUI to control joints

### Simulate in Gazebo

Launch the robot in Gazebo simulator:

```bash
ros2 launch master_bot gazebo.launch.py
```

This will:
- Start Gazebo with an empty world
- Spawn the master_bot robot
- Publish robot transforms

## Changes from ROS 1

The following conversions were made:

1. **Package Format**: Updated from catkin to ament_cmake
2. **Dependencies**: Updated to ROS 2 equivalents (rviz2, gazebo_ros, etc.)
3. **Launch Files**: Converted from XML to Python format
4. **CMakeLists.txt**: Updated to use ament_cmake build system
5. **RViz Config**: Created RViz2 compatible configuration

## Robot Description

The robot URDF includes:
- Base link with inertial properties
- Multiple joints (continuous type)
- Visual and collision meshes
- Material definitions

## Troubleshooting

### Missing Dependencies

If you encounter missing dependencies, install them:

```bash
sudo apt install ros-humble-joint-state-publisher-gui \
                 ros-humble-robot-state-publisher \
                 ros-humble-rviz2 \
                 ros-humble-gazebo-ros-pkgs
```

### URDF Validation

To check if your URDF is valid:

```bash
check_urdf ~/Tank_projects/tank_ws/src/master_bot/urdf/master_bot.urdf
```

### View URDF Structure

To see the kinematic tree:

```bash
urdf_to_graphiz ~/Tank_projects/tank_ws/src/master_bot/urdf/master_bot.urdf
```

## License

BSD

