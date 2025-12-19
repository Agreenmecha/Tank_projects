#!/bin/bash
# Quick workspace sourcing script
# Usage: source source_workspace.sh

source /opt/ros/humble/setup.bash
source /home/aaronjet/Tank_projects/tank_ws/install/setup.bash

echo "✓ ROS2 Humble workspace sourced"
echo "✓ Tank workspace sourced"
echo ""
echo "Quick commands:"
echo "  ros2 launch point_lio mapping_unilidar_l2.launch.py"
echo "  ros2 launch unitree_lidar_ros2 launch.py"
echo "  ros2 topic list"

