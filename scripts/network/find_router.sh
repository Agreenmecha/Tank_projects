#!/bin/bash
# Find Xiaomi Router on the network
# Helps locate router IP address for configuration

echo "=========================================="
echo "Tank Rover - Router Discovery"
echo "=========================================="
echo ""

INTERFACE="eno1"
EXPECTED_NETWORK="192.168.2.0/24"

echo "Scanning network on interface: $INTERFACE"
echo "Network: $EXPECTED_NETWORK"
echo ""

# Check if interface is up
if ! ip link show $INTERFACE | grep -q "state UP"; then
    echo "✗ Interface $INTERFACE is not up"
    echo "Run: sudo nmcli connection up lidar-network"
    exit 1
fi

# Get Jetson IP
JETSON_IP=$(ip addr show $INTERFACE | grep "inet " | awk '{print $2}' | cut -d/ -f1)
echo "Jetson IP: $JETSON_IP"
echo ""

echo "Method 1: Checking ARP table..."
echo "--------------------------------"
arp -a | grep "192.168.2" | grep -v "$JETSON_IP"
echo ""

echo "Method 2: Trying common router IPs..."
echo "--------------------------------------"
for ip in 192.168.2.1 192.168.31.1 192.168.1.1; do
    echo -n "Testing $ip ... "
    if timeout 1 ping -c 1 -W 1 $ip > /dev/null 2>&1; then
        echo "✓ RESPONDS"
        
        # Try to detect if it's a web server
        if timeout 2 curl -s -I http://$ip > /dev/null 2>&1; then
            echo "  → Web interface available at: http://$ip"
        fi
    else
        echo "✗ No response"
    fi
done
echo ""

echo "Method 3: Scanning 192.168.2.0/24 network..."
echo "---------------------------------------------"
echo "This may take 10-20 seconds..."
echo ""

# Check if nmap is installed
if command -v nmap > /dev/null 2>&1; then
    echo "Using nmap for detailed scan:"
    sudo nmap -sn 192.168.2.0/24 | grep -E "Nmap scan report|MAC Address"
else
    echo "nmap not found, using ping sweep:"
    echo "(Install nmap for better results: sudo apt-get install nmap)"
    echo ""
    
    for i in {1..10}; do
        ip="192.168.2.$i"
        if timeout 0.5 ping -c 1 -W 1 $ip > /dev/null 2>&1; then
            echo "✓ $ip is up"
        fi
    done
fi

echo ""
echo "=========================================="
echo "Known Devices:"
echo "=========================================="
echo "  192.168.2.100 - Jetson Orin Nano (this device)"
echo "  192.168.2.62  - LiDAR Front"
echo "  192.168.2.63  - LiDAR Rear"
echo "  192.168.2.1   - Expected router IP (configure this)"
echo ""

echo "=========================================="
echo "Next Steps:"
echo "=========================================="
echo ""
echo "If router found at 192.168.2.x:"
echo "  1. Open browser: firefox http://192.168.2.X"
echo "  2. Login with admin password"
echo "  3. Configure as Access Point"
echo ""
echo "If router at 192.168.31.1 (Xiaomi default):"
echo "  1. Temporarily change Jetson IP:"
echo "     sudo nmcli connection modify lidar-network ipv4.addresses 192.168.31.100/24"
echo "     sudo nmcli connection down lidar-network"
echo "     sudo nmcli connection up lidar-network"
echo ""
echo "  2. Access router: firefox http://192.168.31.1"
echo ""
echo "  3. Configure router with IP 192.168.2.1"
echo ""
echo "  4. Restore Jetson IP:"
echo "     sudo nmcli connection modify lidar-network ipv4.addresses 192.168.2.100/24"
echo "     sudo nmcli connection down lidar-network"
echo "     sudo nmcli connection up lidar-network"
echo ""
echo "See: docs/setup/XIAOMI_ROUTER_AP_SETUP.md for full guide"
echo ""

