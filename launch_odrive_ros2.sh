#!/bin/bash
################################################################################
# Launch ODrive ROS2 Interface Node
################################################################################

echo "=========================================="
echo "Launching ODrive ROS2 Node"
echo "=========================================="
echo ""

cd /home/aaronjet/Tank_projects/tank_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Starting odrive_interface_node..."
echo "Press Ctrl+C to stop"
echo ""

# Run with dialout group for USB access
sg dialout -c "ros2 launch tank_control odrive_interface.launch.py"

