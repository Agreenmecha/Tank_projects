#!/bin/bash
# Point-LIO Verification and Test Script

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

WORKSPACE_DIR="/home/aaronjet/Tank_projects/tank_ws"

echo -e "${YELLOW}========================================="
echo "Point-LIO Verification Script"
echo "=========================================${NC}\n"

cd $WORKSPACE_DIR
source /opt/ros/humble/setup.bash
source install/setup.bash

# Test 1: Check if package is recognized
echo -e "${YELLOW}[1/5] Checking ROS2 package registration...${NC}"
if ros2 pkg list | grep -q "point_lio"; then
    echo -e "${GREEN}✓ point_lio package found${NC}"
else
    echo -e "${RED}✗ point_lio package not found${NC}"
    exit 1
fi

# Test 2: Check executable
echo -e "\n${YELLOW}[2/5] Checking executable...${NC}"
if [ -f "install/point_lio/lib/point_lio/pointlio_mapping" ]; then
    echo -e "${GREEN}✓ pointlio_mapping executable found${NC}"
else
    echo -e "${RED}✗ pointlio_mapping executable not found${NC}"
    exit 1
fi

# Test 3: Check launch files
echo -e "\n${YELLOW}[3/5] Checking launch files...${NC}"
LAUNCH_FILES=(
    "mapping_unilidar_l2.launch.py"
    "mapping_unilidar_l1.launch.py"
    "mapping_avia.launch.py"
    "correct_odom_unilidar_l2.launch.py"
)

for launch_file in "${LAUNCH_FILES[@]}"; do
    if [ -f "install/point_lio/share/point_lio/launch/$launch_file" ]; then
        echo -e "${GREEN}  ✓ $launch_file${NC}"
    else
        echo -e "${RED}  ✗ $launch_file missing${NC}"
    fi
done

# Test 4: Check config files
echo -e "\n${YELLOW}[4/5] Checking config files...${NC}"
CONFIG_FILES=(
    "unilidar_l2.yaml"
    "unilidar_l1.yaml"
    "avia.yaml"
    "mid360.yaml"
)

for config_file in "${CONFIG_FILES[@]}"; do
    if [ -f "install/point_lio/share/point_lio/config/$config_file" ]; then
        echo -e "${GREEN}  ✓ $config_file${NC}"
    else
        echo -e "${RED}  ✗ $config_file missing${NC}"
    fi
done

# Test 5: Check LiDAR driver (L2)
echo -e "\n${YELLOW}[5/5] Checking Unitree L2 LiDAR driver...${NC}"
if ros2 pkg list | grep -q "unitree_lidar_ros2"; then
    echo -e "${GREEN}✓ unitree_lidar_ros2 package found${NC}"
else
    echo -e "${YELLOW}! unitree_lidar_ros2 not found (install unilidar_sdk2 if using L2)${NC}"
fi

echo -e "\n${GREEN}========================================="
echo "✓ Verification Complete!"
echo "=========================================${NC}\n"

echo "Available Commands:"
echo ""
echo "  ${YELLOW}Launch Point-LIO for L2 LiDAR:${NC}"
echo "    ros2 launch point_lio mapping_unilidar_l2.launch.py"
echo ""
echo "  ${YELLOW}Launch L2 LiDAR driver:${NC}"
echo "    ros2 launch unitree_lidar_ros2 launch.py"
echo ""
echo "  ${YELLOW}Check topics:${NC}"
echo "    ros2 topic list"
echo "    ros2 topic hz /unilidar/cloud"
echo "    ros2 topic echo /Odometry --no-arr"
echo ""
echo "  ${YELLOW}View TF tree:${NC}"
echo "    ros2 run tf2_tools view_frames"
echo ""
echo "  ${YELLOW}Launch with RViz:${NC}"
echo "    ros2 launch point_lio mapping_unilidar_l2.launch.py rviz:=true"
echo ""

# Show system info
echo "System Information:"
echo "  Workspace: $WORKSPACE_DIR"
echo "  ROS2 Version: $(ros2 --version 2>&1 | head -n1)"
echo "  Architecture: $(uname -m)"
echo ""

