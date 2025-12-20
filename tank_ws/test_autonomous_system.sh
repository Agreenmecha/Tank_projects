#!/bin/bash
# Quick test script for autonomous system
# Run this to verify all topics after launch

echo "════════════════════════════════════════════════════════════"
echo "  AUTONOMOUS SYSTEM VERIFICATION"
echo "════════════════════════════════════════════════════════════"

sleep 3  # Give nodes time to start

echo ""
echo "1️⃣  Checking LiDAR topics..."
ros2 topic hz /lidar_front/cloud --once 2>&1 | head -3 &
ros2 topic hz /lidar_rear/cloud --once 2>&1 | head -3 &
wait
echo "   ✓ LiDAR check complete"

echo ""
echo "2️⃣  Checking GPS..."
ros2 topic echo /gnss/fix --once 2>&1 | grep -E "latitude|longitude" | head -2
echo "   ✓ GPS check complete"

echo ""
echo "3️⃣  Checking Point-LIO odometry..."
ros2 topic hz /Odometry --once 2>&1 | head -3
echo "   ✓ Point-LIO check complete"

echo ""
echo "4️⃣  Checking Nav2..."
ros2 node list 2>&1 | grep -E "controller_server|planner_server|bt_navigator" | head -3
echo "   ✓ Nav2 check complete"

echo ""
echo "5️⃣  Checking twist_mux..."
ros2 node list 2>&1 | grep twist_mux
ros2 topic echo /twist_mux/selected --once 2>&1 | grep "data:"
echo "   ✓ Twist mux check complete"

echo ""
echo "6️⃣  Checking ODrive..."
ros2 node list 2>&1 | grep odrive
ros2 topic hz /cmd_vel --once 2>&1 | head -3
echo "   ✓ ODrive check complete"

echo ""
echo "════════════════════════════════════════════════════════════"
echo "  ✅ VERIFICATION COMPLETE"
echo "════════════════════════════════════════════════════════════"
echo ""
echo "Next steps:"
echo "  1. On laptop: export ROS_DOMAIN_ID=42"
echo "  2. On laptop: rviz2"
echo "  3. In RViz: Set 2D Goal Pose"
echo "  4. Watch autonomous navigation!"
echo ""

