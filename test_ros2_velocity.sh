#!/bin/bash
################################################################################
# Test ODrive ROS2 Velocity Control
# Run this AFTER launching the odrive_interface_node
################################################################################

echo "=========================================="
echo "ODrive ROS2 Velocity Test"
echo "=========================================="
echo ""

cd /home/aaronjet/Tank_projects/tank_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Make sure odrive_interface_node is running!"
echo "Press Ctrl+C to stop any test"
echo ""

# Check if node is running
if ! ros2 node list | grep -q odrive_interface_node; then
    echo "❌ ERROR: odrive_interface_node is not running!"
    echo "   Start it with: ./launch_odrive_ros2.sh"
    exit 1
fi

echo "✅ ODrive node is running"
echo ""

# Test 1: Forward
echo "Test 1: Moving forward (0.2 m/s for 3 seconds)..."
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
echo "Sending command..."
sleep 3

# Stop
echo "Stopping..."
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 1
echo ""

# Test 2: Rotate
echo "Test 2: Rotating in place (0.5 rad/s for 3 seconds)..."
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
echo "Sending command..."
sleep 3

# Stop
echo "Stopping..."
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 1
echo ""

# Test 3: Combined
echo "Test 3: Forward + rotate (0.3 m/s + 0.3 rad/s for 3 seconds)..."
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"
echo "Sending command..."
sleep 3

# Stop
echo "Stopping..."
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
echo ""

echo "=========================================="
echo "✅ Velocity tests complete!"
echo "=========================================="
echo ""
echo "Check topics:"
echo "  ros2 topic echo /odrive/motor_status"
echo "  ros2 topic echo /odrive/encoder_odom"
echo "  ros2 topic echo /odrive/errors"
echo ""
echo "Or use keyboard teleop:"
echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""

