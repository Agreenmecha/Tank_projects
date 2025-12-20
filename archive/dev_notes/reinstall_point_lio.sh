#!/bin/bash
# Point-LIO ROS2 Clean Reinstallation Script
# For Unitree L2 LiDAR on Ubuntu 22.04 + ROS2 Humble

set -e  # Exit on error

echo "========================================="
echo "Point-LIO ROS2 Reinstallation"
echo "========================================="

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

WORKSPACE_DIR="/home/aaronjet/Tank_projects/tank_ws"

cd $WORKSPACE_DIR

# Step 1: Clean existing build files
echo -e "\n${YELLOW}[1/6] Cleaning existing build files...${NC}"
rm -rf build/point_lio install/point_lio log/*/point_lio
echo -e "${GREEN}✓ Cleaned build files${NC}"

# Step 2: Verify dependencies
echo -e "\n${YELLOW}[2/6] Verifying dependencies...${NC}"

# Check ROS2 packages
if ! dpkg -l | grep -q "ros-humble-pcl-ros"; then
    echo "Installing ros-humble-pcl-ros..."
    sudo apt-get install -y ros-humble-pcl-ros
fi

if ! dpkg -l | grep -q "ros-humble-pcl-conversions"; then
    echo "Installing ros-humble-pcl-conversions..."
    sudo apt-get install -y ros-humble-pcl-conversions
fi

if ! dpkg -l | grep -q "ros-humble-visualization-msgs"; then
    echo "Installing ros-humble-visualization-msgs..."
    sudo apt-get install -y ros-humble-visualization-msgs
fi

if ! dpkg -l | grep -q "libeigen3-dev"; then
    echo "Installing libeigen3-dev..."
    sudo apt-get install -y libeigen3-dev
fi

echo -e "${GREEN}✓ All dependencies verified${NC}"

# Step 3: Verify unilidar_sdk2 is installed
echo -e "\n${YELLOW}[3/6] Verifying Unitree LiDAR SDK2...${NC}"
if [ ! -d "src/external/unilidar_sdk2" ]; then
    echo "ERROR: unilidar_sdk2 not found!"
    echo "Please install it first:"
    echo "  cd $WORKSPACE_DIR/src/external"
    echo "  git clone https://github.com/unitreerobotics/unilidar_sdk2.git"
    exit 1
fi

if [ ! -d "install/unitree_lidar_ros2" ]; then
    echo "Building unilidar_sdk2..."
    colcon build --packages-select unitree_lidar_ros2 unitree_lidar_sdk --symlink-install
fi

echo -e "${GREEN}✓ Unitree LiDAR SDK2 verified${NC}"

# Step 4: Source the workspace to ensure unilidar_sdk2 is available
echo -e "\n${YELLOW}[4/6] Sourcing workspace...${NC}"
source /opt/ros/humble/setup.bash
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
fi
echo -e "${GREEN}✓ Workspace sourced${NC}"

# Step 5: Build Point-LIO
echo -e "\n${YELLOW}[5/6] Building Point-LIO...${NC}"
colcon build --packages-select point_lio --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Point-LIO built successfully${NC}"
else
    echo "ERROR: Build failed!"
    exit 1
fi

# Step 6: Final verification
echo -e "\n${YELLOW}[6/6] Final verification...${NC}"
source install/setup.bash

# Check if launch files exist
if [ -f "install/point_lio/share/point_lio/launch/mapping_unilidar_l2.launch.py" ]; then
    echo -e "${GREEN}✓ Launch files installed${NC}"
else
    echo "ERROR: Launch files not found!"
    exit 1
fi

# Check if config files exist
if [ -f "install/point_lio/share/point_lio/config/unilidar_l2.yaml" ]; then
    echo -e "${GREEN}✓ Config files installed${NC}"
else
    echo "ERROR: Config files not found!"
    exit 1
fi

echo -e "\n${GREEN}========================================="
echo "✓ Point-LIO installation complete!"
echo "=========================================${NC}"
echo ""
echo "To use Point-LIO:"
echo "  1. Source the workspace: source $WORKSPACE_DIR/install/setup.bash"
echo "  2. Launch for L2 LiDAR: ros2 launch point_lio mapping_unilidar_l2.launch.py"
echo ""
echo "Available launch files:"
echo "  - mapping_unilidar_l2.launch.py    (for Unitree L2)"
echo "  - mapping_unilidar_l1.launch.py    (for Unitree L1)"
echo "  - mapping_avia.launch.py           (for Livox Avia)"
echo "  - mapping_mid360.launch.py         (for Livox Mid-360)"
echo "  - mapping_velody16.launch.py       (for Velodyne 16)"
echo "  - mapping_ouster64.launch.py       (for Ouster 64)"
echo ""

