#!/bin/bash
# Setup network for dual Unitree L2 LiDARs
# Run with sudo

set -e

echo "=========================================="
echo "Dual Unitree L2 LiDAR Network Setup"
echo "=========================================="
echo ""

JETSON_IP="192.168.1.100"
INTERFACE="eno1"

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "Please run with sudo"
    exit 1
fi

echo "Configuring $INTERFACE with IP $JETSON_IP..."

# Create NetworkManager connection for LiDAR network
nmcli connection add \
    type ethernet \
    con-name lidar-network \
    ifname $INTERFACE \
    ipv4.method manual \
    ipv4.addresses $JETSON_IP/24 \
    ipv4.gateway 192.168.1.1 \
    autoconnect yes

# Activate the connection
nmcli connection up lidar-network

echo ""
echo "✓ Network configured successfully"
echo ""
echo "Network Configuration:"
echo "  Jetson IP:     $JETSON_IP"
echo "  Interface:     $INTERFACE"
echo "  Subnet:        192.168.1.0/24"
echo ""
echo "Expected LiDAR Configuration:"
echo "  Front LiDAR:   192.168.1.62 → $JETSON_IP:6201"
echo "  Rear LiDAR:    192.168.1.63 → $JETSON_IP:6202"
echo ""
echo "=========================================="
echo "Next Steps:"
echo "=========================================="
echo ""
echo "1. Connect ONE LiDAR via UART to configure:"
echo "   - Use Unilidar 2 software on Windows PC"
echo "   - Connect via serial converter"
echo "   - Set Rear LiDAR to IP 192.168.1.63"
echo "   - Set Target IP to $JETSON_IP"
echo "   - Set Target Port to 6202"
echo ""
echo "2. Connect both LiDARs to network switch"
echo ""
echo "3. Test connection:"
echo "   ping 192.168.1.62  # Front"
echo "   ping 192.168.1.63  # Rear"
echo ""
echo "4. Verify UDP data reception:"
echo "   sudo tcpdump -i $INTERFACE udp port 6201 -n"
echo "   sudo tcpdump -i $INTERFACE udp port 6202 -n"
echo ""

