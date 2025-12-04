#!/bin/bash
# Test script for ZED-F9P GNSS

cd /home/aaronjet/Tank_projects/tank_ws
source install/setup.bash

echo "=========================================="
echo "ZED-F9P GNSS Test"
echo "=========================================="
echo ""

# Check if node is running
echo "1. Checking GNSS node..."
if ros2 node list | grep -q gnss_node; then
    echo -e "\033[0;32m✓\033[0m GNSS node is running"
else
    echo -e "\033[0;31m❌\033[0m GNSS node is NOT running"
    echo "   Start with: ros2 launch tank_sensors gnss.launch.py"
    exit 1
fi
echo ""

# List available topics
echo "2. Available GNSS topics:"
ros2 topic list | grep "ubx" | head -10
echo "   ... ($(ros2 topic list | grep ubx | wc -l) total topics)"
echo ""

# Test key topics
echo "3. Testing key topics..."

echo "   a) NAV-PVT (Position/Velocity/Time):"
TIMEOUT=5
if timeout $TIMEOUT ros2 topic hz /ubx_nav_pvt 2>&1 | grep -q "average rate"; then
    HZ=$(timeout $TIMEOUT ros2 topic hz /ubx_nav_pvt 2>&1 | grep "average rate" | awk '{print $3}')
    echo -e "      \033[0;32m✓\033[0m Publishing at ${HZ} Hz"
else
    echo -e "      \033[0;33m⚠\033[0m Not publishing (may need GNSS fix)"
fi

echo "   b) NAV-HP-POS-LLH (High-Precision Lat/Lon):"
if timeout $TIMEOUT ros2 topic hz /ubx_nav_hp_pos_llh 2>&1 | grep -q "average rate"; then
    HZ=$(timeout $TIMEOUT ros2 topic hz /ubx_nav_hp_pos_llh 2>&1 | grep "average rate" | awk '{print $3}')
    echo -e "      \033[0;32m✓\033[0m Publishing at ${HZ} Hz"
else
    echo -e "      \033[0;33m⚠\033[0m Not publishing (may need GNSS fix)"
fi

echo "   c) NAV-DOP (Dilution of Precision):"
if timeout $TIMEOUT ros2 topic hz /ubx_nav_dop 2>&1 | grep -q "average rate"; then
    HZ=$(timeout $TIMEOUT ros2 topic hz /ubx_nav_dop 2>&1 | grep "average rate" | awk '{print $3}')
    echo -e "      \033[0;32m✓\033[0m Publishing at ${HZ} Hz"
else
    echo -e "      \033[0;33m⚠\033[0m Not publishing (may need GNSS fix)"
fi
echo ""

# Sample position data
echo "4. Sample position data (waiting up to 10s for fix)..."
POSITION=$(timeout 10 ros2 topic echo /ubx_nav_hp_pos_llh --once 2>&1)
if [ $? -eq 0 ] && [ -n "$POSITION" ]; then
    echo -e "\033[0;32m✓\033[0m Position received!"
    echo "$POSITION" | grep -E "lat:|lon:|height:|accuracy" | head -6
else
    echo -e "\033[0;33m⚠\033[0m No position data yet"
    echo "   GNSS may still be acquiring satellites..."
    echo "   Check antenna placement (needs clear sky view)"
fi
echo ""

echo "=========================================="
echo "GNSS Status Summary:"
NODE_STATUS=$(ros2 node list | grep -q gnss_node && echo "Running" || echo "Not Running")
TOPIC_COUNT=$(ros2 topic list | grep ubx | wc -l)
echo "  Node: $NODE_STATUS"
echo "  Topics: $TOPIC_COUNT available"
echo ""
echo "For live monitoring: rviz2"
echo "For position echo: ros2 topic echo /ubx_nav_hp_pos_llh"
echo "=========================================="

