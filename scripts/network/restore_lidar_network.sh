#!/bin/bash
# Restore lidar network configuration after accessing router on default network

set -e

echo "=========================================="
echo "Restore LiDAR Network Configuration"
echo "=========================================="
echo ""

CONNECTION="lidar-network"
ORIGINAL_NETWORK="192.168.2.100/24"

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "Please run with sudo"
    exit 1
fi

echo "Restoring Jetson IP to: $ORIGINAL_NETWORK"
echo ""

nmcli connection modify $CONNECTION ipv4.addresses $ORIGINAL_NETWORK
nmcli connection down $CONNECTION
sleep 1
nmcli connection up $CONNECTION

echo "✓ Network configuration restored"
echo ""

# Verify connectivity
sleep 2
echo "Verifying connectivity..."
echo ""

if ping -c 2 -W 2 192.168.2.62 > /dev/null 2>&1; then
    echo "✓ LiDAR Front (192.168.2.62) - OK"
else
    echo "✗ LiDAR Front (192.168.2.62) - Not responding"
fi

if ping -c 2 -W 2 192.168.2.63 > /dev/null 2>&1; then
    echo "✓ LiDAR Rear (192.168.2.63) - OK"
else
    echo "✗ LiDAR Rear (192.168.2.63) - Not responding"
fi

if ping -c 2 -W 2 192.168.2.1 > /dev/null 2>&1; then
    echo "✓ Router (192.168.2.1) - OK"
    echo ""
    echo "Router web interface: http://192.168.2.1"
else
    echo "⚠️  Router (192.168.2.1) - Not responding"
    echo "   If you just configured it, wait 1-2 minutes for reboot"
fi

echo ""
echo "Current network status:"
ip addr show eno1 | grep "inet "

echo ""
echo "=========================================="
echo "✓ Done"
echo "=========================================="
echo ""

