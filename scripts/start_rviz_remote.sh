#!/bin/bash
# Start RViz2 for remote visualization
# Run this on the Jetson (via SSH with X11 forwarding)

cd ~/Tank_projects/tank_ws
source install/setup.bash

echo "=========================================="
echo "Starting RViz2 for Tank Sensors"
echo "=========================================="
echo ""
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "Available topics: $(ros2 topic list | wc -l)"
echo ""
echo "RViz Configuration:"
echo "  1. Fixed Frame: Set to 'lidar_front'"
echo "  2. Add PointCloud2 displays:"
echo "     - /lidar_front/pointcloud"
echo "     - /lidar_rear/pointcloud"
echo "  3. Add TF display to see coordinate frames"
echo ""
echo "Starting RViz2..."
echo ""

rviz2

