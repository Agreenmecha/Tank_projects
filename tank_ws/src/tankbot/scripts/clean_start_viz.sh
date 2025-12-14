#!/bin/bash
#
# Clean start script for remote visualization
# Kills existing processes and restarts robot_state_publisher, joint_state_publisher, and RViz
#
# Usage:
#   ./clean_start_viz.sh
#   or
#   bash clean_start_viz.sh
#

# Don't use set -e, we want to handle errors gracefully

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== Clean Start for Remote Visualization ===${NC}"
echo ""

# Get the workspace directory (assuming script is in tankbot/scripts/)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_DIR="$( cd "$SCRIPT_DIR/../../.." && pwd )"

echo -e "${YELLOW}Workspace: $WS_DIR${NC}"
echo ""

# Step 1: Kill existing processes
echo -e "${YELLOW}[1/4] Killing existing processes...${NC}"
pkill -f "robot_state_publisher" 2>/dev/null && echo "  Killed robot_state_publisher" || echo "  No robot_state_publisher processes found"
pkill -f "joint_state_publisher" 2>/dev/null && echo "  Killed joint_state_publisher" || echo "  No joint_state_publisher processes found"
pkill -f "rviz2" 2>/dev/null && echo "  Killed rviz2" || echo "  No rviz2 processes found"
pkill -f "world_to_base_footprint" 2>/dev/null && echo "  Killed world_to_base_footprint" || echo "  No world_to_base_footprint processes found"

# Wait a moment for processes to fully terminate
sleep 1

# Verify processes are killed
if pgrep -f "robot_state_publisher|joint_state_publisher|rviz2" > /dev/null; then
    echo -e "${RED}Warning: Some processes are still running. Attempting force kill...${NC}"
    pkill -9 -f "robot_state_publisher|joint_state_publisher|rviz2" 2>/dev/null || true
    sleep 1
fi

echo -e "${GREEN}  ✓ Processes killed${NC}"
echo ""

# Step 2: Source ROS and workspace
echo -e "${YELLOW}[2/4] Sourcing ROS and workspace...${NC}"
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"
echo -e "${GREEN}  ✓ Environment sourced${NC}"
echo ""

# Step 3: Verify URDF
echo -e "${YELLOW}[3/4] Verifying URDF...${NC}"
URDF_PATH="$WS_DIR/install/tankbot/share/tankbot/urdf/tankbot.urdf"
if [ ! -f "$URDF_PATH" ]; then
    echo -e "${RED}Error: URDF file not found at $URDF_PATH${NC}"
    echo "  Building package..."
    cd "$WS_DIR"
    colcon build --packages-select tankbot
    source install/setup.bash
fi

if command -v check_urdf > /dev/null 2>&1; then
    if check_urdf "$URDF_PATH" > /dev/null 2>&1; then
        echo -e "${GREEN}  ✓ URDF is valid${NC}"
    else
        echo -e "${RED}  ✗ URDF validation failed${NC}"
        check_urdf "$URDF_PATH"
        exit 1
    fi
else
    echo -e "${YELLOW}  ⚠ check_urdf not found, skipping validation${NC}"
fi
echo ""

# Step 4: Launch remote_viz
echo -e "${YELLOW}[4/4] Launching remote visualization...${NC}"
echo ""
echo -e "${GREEN}Starting nodes:${NC}"
echo "  - robot_state_publisher"
echo "  - joint_state_publisher"
echo "  - world_to_base_footprint (static transform)"
echo "  - rviz2"
echo ""
echo -e "${YELLOW}Press Ctrl+C to stop all nodes${NC}"
echo ""

cd "$WS_DIR"
ros2 launch tankbot remote_viz.launch.py

