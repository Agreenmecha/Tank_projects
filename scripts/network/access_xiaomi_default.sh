#!/bin/bash
# Temporarily configure Jetson to access Xiaomi router on default network
# This allows you to configure a brand new or reset Xiaomi router

set -e

echo "=========================================="
echo "Access Xiaomi Router (Default Network)"
echo "=========================================="
echo ""

CONNECTION="lidar-network"
INTERFACE="eno1"
DEFAULT_NETWORK="192.168.31.100/24"
ORIGINAL_NETWORK="192.168.2.100/24"

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "Please run with sudo"
    exit 1
fi

echo "⚠️  WARNING: This will temporarily disconnect LiDARs!"
echo ""
echo "This script will:"
echo "  1. Change Jetson IP from 192.168.2.100 to 192.168.31.100"
echo "  2. Allow you to access router at http://192.168.31.1"
echo "  3. Provide instructions to restore original network"
echo ""
read -p "Continue? (y/N) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Cancelled."
    exit 0
fi

echo ""
echo "Step 1: Backing up current configuration..."
nmcli connection show $CONNECTION > /tmp/lidar-network-backup.txt
echo "✓ Backup saved to /tmp/lidar-network-backup.txt"

echo ""
echo "Step 2: Changing Jetson IP to Xiaomi default network..."
nmcli connection modify $CONNECTION ipv4.addresses $DEFAULT_NETWORK
nmcli connection down $CONNECTION
sleep 1
nmcli connection up $CONNECTION

echo "✓ Network changed"

echo ""
echo "Step 3: Testing router connectivity..."
sleep 2

if ping -c 2 -W 2 192.168.31.1 > /dev/null 2>&1; then
    echo "✓ Router found at 192.168.31.1"
    
    # Try to open browser
    echo ""
    echo "Opening router web interface..."
    
    # Detect available browser
    if command -v firefox > /dev/null 2>&1; then
        firefox http://192.168.31.1 &
    elif command -v chromium-browser > /dev/null 2>&1; then
        chromium-browser http://192.168.31.1 &
    elif command -v google-chrome > /dev/null 2>&1; then
        google-chrome http://192.168.31.1 &
    else
        echo "No browser found. Manually open: http://192.168.31.1"
    fi
    
    echo ""
    echo "=========================================="
    echo "✓ Ready to Configure Router"
    echo "=========================================="
    echo ""
    echo "Router Web Interface: http://192.168.31.1"
    echo "Default URL: http://miwifi.com"
    echo ""
    echo "Configuration Steps:"
    echo "--------------------"
    echo "1. Login (check router label for password)"
    echo ""
    echo "2. Go to: Settings → Network Settings"
    echo ""
    echo "3. Set Router IP:"
    echo "   - IP Address: 192.168.2.1"
    echo "   - Subnet Mask: 255.255.255.0"
    echo "   - Gateway: 192.168.2.100"
    echo ""
    echo "4. Disable DHCP Server"
    echo ""
    echo "5. Configure WiFi:"
    echo "   - SSID: Tank_Rover_AP"
    echo "   - Password: [your choice]"
    echo "   - Security: WPA2/WPA3"
    echo ""
    echo "6. Save and Reboot Router"
    echo ""
else
    echo "✗ Router not found at 192.168.31.1"
    echo ""
    echo "Possible reasons:"
    echo "  - Router is not powered on"
    echo "  - Router is not connected to eno1"
    echo "  - Router has different default IP"
    echo "  - Router is already configured"
    echo ""
fi

echo ""
echo "=========================================="
echo "⚠️  IMPORTANT: Restore Network After Configuration"
echo "=========================================="
echo ""
echo "After configuring the router, run this command:"
echo ""
echo "  sudo nmcli connection modify $CONNECTION ipv4.addresses $ORIGINAL_NETWORK"
echo "  sudo nmcli connection down $CONNECTION"
echo "  sudo nmcli connection up $CONNECTION"
echo ""
echo "Or run the restore script:"
echo "  sudo /home/aaronjet/Tank_projects/scripts/network/restore_lidar_network.sh"
echo ""
echo "Press Enter when router configuration is complete..."
read

echo ""
echo "Restoring original network configuration..."
nmcli connection modify $CONNECTION ipv4.addresses $ORIGINAL_NETWORK
nmcli connection down $CONNECTION
sleep 1
nmcli connection up $CONNECTION

echo ""
echo "✓ Network restored to 192.168.2.100/24"
echo ""

# Verify
sleep 2
echo "Verifying connectivity..."
if ping -c 2 192.168.2.62 > /dev/null 2>&1; then
    echo "✓ LiDAR Front (192.168.2.62) - OK"
else
    echo "✗ LiDAR Front - Not responding"
fi

if ping -c 2 192.168.2.63 > /dev/null 2>&1; then
    echo "✓ LiDAR Rear (192.168.2.63) - OK"
else
    echo "✗ LiDAR Rear - Not responding"
fi

if ping -c 2 192.168.2.1 > /dev/null 2>&1; then
    echo "✓ Router (192.168.2.1) - OK"
    echo ""
    echo "🎉 Success! Router is now accessible at http://192.168.2.1"
else
    echo "⚠️  Router (192.168.2.1) - Not responding yet"
    echo "   Wait 1-2 minutes for router to reboot, then check again"
fi

echo ""
echo "=========================================="
echo "✓ Configuration Complete"
echo "=========================================="
echo ""

