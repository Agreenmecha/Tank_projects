#!/bin/bash
################################################################################
# Install Sensor Drivers
# Unitree L2 LiDAR and u-blox ZED-F9P GNSS
################################################################################

set -e

echo "Installing sensor drivers..."
echo ""

# Source ROS 2
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "✗ ROS 2 not found. Please run 03_install_ros2.sh first."
    exit 1
fi

# Create workspace if it doesn't exist
WORKSPACE_DIR=~/Tank_projects/tank_ws
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo "Creating workspace directory..."
    mkdir -p "$WORKSPACE_DIR/src/external"
fi

cd "$WORKSPACE_DIR/src/external"

# Install Unitree L2 LiDAR driver
echo ""
echo "Installing Unitree L2 LiDAR driver..."
if [ ! -d "unilidar_sdk2" ]; then
    git clone https://github.com/unitreerobotics/unilidar_sdk2.git
    echo "✓ Unitree LiDAR driver cloned"
else
    echo "⚠ Unitree LiDAR driver already exists, skipping clone"
fi

# Install u-blox ZED-F9P GNSS driver
echo ""
echo "Installing u-blox ZED-F9P GNSS driver..."
if [ ! -d "ublox_dgnss" ]; then
    git clone https://github.com/aussierobots/ublox_dgnss.git
    echo "✓ u-blox GNSS driver cloned"
else
    echo "⚠ u-blox GNSS driver already exists, skipping clone"
fi

# Add GNSS udev rules
echo ""
echo "Adding GNSS udev rules..."
sudo bash -c 'cat > /etc/udev/rules.d/99-ublox.rules << EOF
# u-blox GNSS receivers
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", MODE="0666", SYMLINK+="ublox"
SUBSYSTEM=="usb", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", MODE="0666"
EOF'

sudo udevadm control --reload-rules
sudo udevadm trigger

echo "✓ GNSS udev rules added"

# Build sensor drivers
echo ""
echo "Building sensor drivers (this may take 5-10 minutes)..."
cd "$WORKSPACE_DIR"

# Build Unitree LiDAR driver
echo ""
echo "Building Unitree LiDAR driver..."
colcon build --packages-select unitree_lidar_ros2 --symlink-install

# Build u-blox GNSS driver
echo ""
echo "Building u-blox GNSS driver..."
colcon build --packages-select ublox_ubx_msgs ublox_ubx_interfaces ublox_dgnss_node --symlink-install

# Source the workspace
source "$WORKSPACE_DIR/install/setup.bash"

echo ""
echo "✓ Sensor drivers installed and built successfully"
echo ""
echo "Installed drivers:"
echo "  • Unitree L2 LiDAR (unilidar_sdk2)"
echo "  • u-blox ZED-F9P GNSS (ublox_dgnss)"
echo ""
echo "Workspace: $WORKSPACE_DIR"

