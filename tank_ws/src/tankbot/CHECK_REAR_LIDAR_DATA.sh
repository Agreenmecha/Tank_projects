#!/bin/bash
# Quick script to check if rear LiDAR is publishing data

export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source ~/Tank_projects/tank_ws/install/setup.bash

echo "=== Checking Rear LiDAR Topic ==="
echo ""

echo "Topic info:"
ros2 topic info /lidar_rear/cloud

echo ""
echo "Message header and dimensions:"
ros2 topic echo /lidar_rear/cloud --once 2>&1 | grep -E "frame_id|width|height|point_step" | head -5

echo ""
echo "Checking if width > 0 (has data):"
WIDTH=$(ros2 topic echo /lidar_rear/cloud --once 2>&1 | grep "width:" | awk '{print $2}')
if [ -z "$WIDTH" ]; then
    echo "  ERROR: Could not read width from topic"
elif [ "$WIDTH" = "0" ]; then
    echo "  WARNING: width = 0 (empty point cloud - no data)"
else
    echo "  OK: width = $WIDTH (point cloud has data)"
fi

echo ""
echo "Message rate:"
timeout 3 ros2 topic hz /lidar_rear/cloud 2>&1 | head -5

