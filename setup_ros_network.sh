#!/bin/bash
# ROS 2 Network Configuration for Jetson
# Allows desktop to visualize Jetson's ROS 2 topics

echo "=========================================="
echo "ROS 2 Network Setup for Remote Visualization"
echo "=========================================="
echo ""

# Get Jetson IP
JETSON_IP=$(ip -4 addr show wlP1p1s0 | grep -oP '(?<=inet\s)\d+(\.\d+){3}')
echo "Jetson IP: $JETSON_IP"
echo ""

# Add to bashrc if not already there
if ! grep -q "ROS_DOMAIN_ID" ~/.bashrc; then
    echo "Adding ROS_DOMAIN_ID to ~/.bashrc"
    echo "" >> ~/.bashrc
    echo "# ROS 2 Network Configuration" >> ~/.bashrc
    echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
fi

echo "✓ Configuration added to ~/.bashrc"
echo ""
echo "=========================================="
echo "Desktop Setup Instructions:"
echo "=========================================="
echo ""
echo "1. Install ROS 2 Humble on your desktop"
echo "2. Add to desktop's ~/.bashrc:"
echo "   export ROS_DOMAIN_ID=42"
echo "   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
echo ""
echo "3. On desktop, run:"
echo "   source ~/.bashrc"
echo "   rviz2"
echo ""
echo "4. In RViz:"
echo "   - Set Fixed Frame: lidar_front"
echo "   - Add → PointCloud2 → /lidar_front/pointcloud"
echo "   - Add → PointCloud2 → /lidar_rear/pointcloud"
echo ""
echo "=========================================="
echo "Firewall Configuration (if needed):"
echo "=========================================="
echo ""
echo "On Jetson:"
echo "  sudo ufw allow from 192.168.1.0/24"
echo ""
echo "On Desktop:"
echo "  sudo ufw allow from 192.168.1.0/24"
echo ""
echo "=========================================="
echo "Testing Connection:"
echo "=========================================="
echo ""
echo "On Desktop, after launching RViz:"
echo "  ros2 topic list"
echo ""
echo "Should see Jetson topics like:"
echo "  /lidar_front/pointcloud"
echo "  /lidar_rear/pointcloud"
echo "  /lidar_front/imu"
echo "  /lidar_rear/imu"
echo ""

