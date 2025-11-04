#!/bin/bash
################################################################################
# Setup Project Workspace
# Create tank_sensors package and configuration files
################################################################################

set -e

echo "Setting up project workspace..."
echo ""

# Source ROS 2
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "✗ ROS 2 not found. Please run 03_install_ros2.sh first."
    exit 1
fi

WORKSPACE_DIR=~/Tank_projects/tank_ws
PROJECT_DIR=~/Tank_projects

# Create tank_sensors package if it doesn't exist
if [ ! -d "$WORKSPACE_DIR/src/tank_sensors" ]; then
    echo "Creating tank_sensors package..."
    cd "$WORKSPACE_DIR/src"
    ros2 pkg create tank_sensors \
        --build-type ament_cmake \
        --dependencies rclcpp sensor_msgs geometry_msgs
    
    # Create directories
    mkdir -p "$WORKSPACE_DIR/src/tank_sensors/launch"
    mkdir -p "$WORKSPACE_DIR/src/tank_sensors/config"
    
    echo "✓ tank_sensors package created"
else
    echo "⚠ tank_sensors package already exists"
fi

# Copy launch files from main project (if they exist)
echo ""
echo "Setting up launch files..."
if [ -f "$WORKSPACE_DIR/src/tank_sensors/launch/dual_lidar.launch.py" ]; then
    echo "✓ dual_lidar.launch.py already exists"
else
    echo "⚠ dual_lidar.launch.py needs to be created manually"
fi

if [ -f "$WORKSPACE_DIR/src/tank_sensors/launch/gnss.launch.py" ]; then
    echo "✓ gnss.launch.py already exists"
else
    echo "⚠ gnss.launch.py needs to be created manually"
fi

# Build tank_sensors package
echo ""
echo "Building tank_sensors package..."
cd "$WORKSPACE_DIR"
colcon build --packages-select tank_sensors --symlink-install

# Add workspace sourcing to bashrc
echo ""
echo "Configuring environment..."
if ! grep -q 'tank_ws/install/setup.bash' ~/.bashrc; then
    echo '' >> ~/.bashrc
    echo '# Tank project workspace' >> ~/.bashrc
    echo 'source ~/Tank_projects/tank_ws/install/setup.bash' >> ~/.bashrc
    echo "Added workspace sourcing to ~/.bashrc"
fi

# Source the workspace
source "$WORKSPACE_DIR/install/setup.bash"

echo ""
echo "✓ Project workspace setup complete"
echo ""
echo "Workspace structure:"
echo "  $WORKSPACE_DIR/"
echo "  ├── src/"
echo "  │   ├── tank_sensors/      # Tank-specific sensors package"
echo "  │   └── external/"
echo "  │       ├── unilidar_sdk2/  # Unitree LiDAR driver"
echo "  │       └── ublox_dgnss/    # u-blox GNSS driver"
echo "  ├── build/"
echo "  └── install/"

