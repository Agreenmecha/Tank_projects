#!/bin/bash
# Setup script for laptop to connect to Jetson ROS2
# Run this on your LAPTOP (not on Jetson)

echo "=========================================="
echo "ROS2 Remote Visualization Setup"
echo "For Laptop to Connect to Jetson"
echo "=========================================="
echo ""

# Check if ROS2 is installed
if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "❌ ROS2 Humble not found!"
    echo ""
    echo "Install ROS2 Humble on your laptop:"
    echo "  sudo apt update"
    echo "  sudo apt install ros-humble-desktop"
    echo ""
    exit 1
fi

echo "✅ ROS2 Humble found"
echo ""

# Check if ROS_DOMAIN_ID is set
if ! grep -q "ROS_DOMAIN_ID=42" ~/.bashrc 2>/dev/null; then
    echo "Adding ROS_DOMAIN_ID=42 to ~/.bashrc..."
    echo "" >> ~/.bashrc
    echo "# ROS2 Remote Visualization - Connect to Jetson" >> ~/.bashrc
    echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
    echo "✅ Added to ~/.bashrc"
    echo ""
    echo "⚠️  Run: source ~/.bashrc"
    echo ""
else
    echo "✅ ROS_DOMAIN_ID=42 already in ~/.bashrc"
fi

echo ""
echo "=========================================="
echo "Setup Complete!"
echo "=========================================="
echo ""
echo "On your laptop, run:"
echo "  1. source ~/.bashrc"
echo "  2. rviz2"
echo ""
echo "In RViz2:"
echo "  - Fixed Frame: 'lidar_front'"
echo "  - Add → By topic → /lidar_front/pointcloud → PointCloud2"
echo "  - Add → By topic → /lidar_rear/pointcloud → PointCloud2"
echo ""
echo "Verify connection:"
echo "  ros2 topic list  # Should show Jetson topics"
echo ""

