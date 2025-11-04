#!/bin/bash
# Quick test script for dual Unitree L2 LiDARs
# Verifies both LiDARs are connected and streaming

set -e

echo "=========================================="
echo "Dual LiDAR Network Test"
echo "=========================================="
echo ""

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test network interface
echo "1. Checking network interface..."
if ip addr show eno1 | grep -q "192.168.2.100"; then
    echo -e "${GREEN}✓${NC} Jetson IP: 192.168.2.100"
else
    echo -e "${RED}✗${NC} Jetson network not configured!"
    exit 1
fi
echo ""

# Test Front LiDAR
echo "2. Testing Front LiDAR (192.168.2.62)..."
if ping -c 2 -W 1 192.168.2.62 > /dev/null 2>&1; then
    echo -e "${GREEN}✓${NC} Front LiDAR reachable"
else
    echo -e "${RED}✗${NC} Front LiDAR not responding"
    exit 1
fi

# Test Rear LiDAR
echo "3. Testing Rear LiDAR (192.168.2.63)..."
if ping -c 2 -W 1 192.168.2.63 > /dev/null 2>&1; then
    echo -e "${GREEN}✓${NC} Rear LiDAR reachable"
else
    echo -e "${RED}✗${NC} Rear LiDAR not responding"
    exit 1
fi
echo ""

# Test UDP data streams
echo "4. Checking UDP data streams..."
echo -e "${YELLOW}   Capturing 20 packets (5 seconds)...${NC}"
CAPTURE=$(sudo timeout 5 tcpdump -i eno1 -n "udp and (port 6201 or port 6202)" -c 20 2>&1)

FRONT_COUNT=$(echo "$CAPTURE" | grep -c "192.168.2.62.6101 > 192.168.2.100.6201" || true)
REAR_COUNT=$(echo "$CAPTURE" | grep -c "192.168.2.63.6101 > 192.168.2.100.6202" || true)

if [ $FRONT_COUNT -gt 0 ]; then
    echo -e "${GREEN}✓${NC} Front LiDAR streaming ($FRONT_COUNT packets)"
else
    echo -e "${RED}✗${NC} Front LiDAR not streaming"
fi

if [ $REAR_COUNT -gt 0 ]; then
    echo -e "${GREEN}✓${NC} Rear LiDAR streaming ($REAR_COUNT packets)"
else
    echo -e "${RED}✗${NC} Rear LiDAR not streaming"
fi

echo ""
echo "=========================================="
if [ $FRONT_COUNT -gt 0 ] && [ $REAR_COUNT -gt 0 ]; then
    echo -e "${GREEN}SUCCESS: Both LiDARs operational!${NC}"
    echo ""
    echo "Network Status:"
    echo "  Front: 192.168.2.62 → :6201 ($FRONT_COUNT pkts)"
    echo "  Rear:  192.168.2.63 → :6202 ($REAR_COUNT pkts)"
else
    echo -e "${RED}FAILED: One or more LiDARs not working${NC}"
    exit 1
fi
echo "=========================================="
echo ""
echo "Next: ros2 launch tank_sensors lidar.launch.py"
echo ""

