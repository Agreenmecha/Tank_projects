#!/bin/bash
################################################################################
# Configure Hardware
# Network setup for LiDARs and utility scripts
################################################################################

set -e

echo "Configuring hardware and network..."
echo ""

PROJECT_DIR=~/Tank_projects

# Configure LiDAR network
echo "Setting up LiDAR network configuration..."

# Check if lidar-network connection already exists
if nmcli connection show lidar-network &>/dev/null; then
    echo "⚠ LiDAR network connection already exists"
    read -p "Reconfigure? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        sudo nmcli connection delete lidar-network
    else
        echo "Skipping LiDAR network configuration"
        SKIP_NETWORK=true
    fi
fi

if [ "$SKIP_NETWORK" != "true" ]; then
    echo "Creating LiDAR network connection..."
    sudo nmcli connection add \
        type ethernet \
        con-name lidar-network \
        ifname eno1 \
        ipv4.method manual \
        ipv4.addresses 192.168.2.100/24 \
        autoconnect yes
    
    echo "✓ LiDAR network configured (192.168.2.100/24)"
else
    echo "Network configuration skipped"
fi

# Copy utility scripts to project directory
echo ""
echo "Creating utility scripts..."

# LiDAR test script
if [ ! -f "$PROJECT_DIR/test_dual_lidar.sh" ]; then
    cat > "$PROJECT_DIR/test_dual_lidar.sh" << 'EOF'
#!/bin/bash
# Quick test script for dual LiDAR connectivity
echo "Testing dual LiDAR network..."
echo ""
echo "Jetson IP: $(ip addr show eno1 | grep 'inet ' | awk '{print $2}')"
echo ""
echo -n "Front LiDAR (192.168.2.62): "
ping -c 1 -W 1 192.168.2.62 &>/dev/null && echo "✓ Reachable" || echo "✗ Not reachable"
echo -n "Rear LiDAR (192.168.2.63): "
ping -c 1 -W 1 192.168.2.63 &>/dev/null && echo "✓ Reachable" || echo "✗ Not reachable"
EOF
    chmod +x "$PROJECT_DIR/test_dual_lidar.sh"
    echo "✓ Created test_dual_lidar.sh"
fi

# GNSS test script
if [ ! -f "$PROJECT_DIR/test_gnss.sh" ]; then
    cat > "$PROJECT_DIR/test_gnss.sh" << 'EOF'
#!/bin/bash
# Quick test script for GNSS
source ~/Tank_projects/tank_ws/install/setup.bash
echo "Testing GNSS..."
echo ""
echo -n "GNSS device: "
ls /dev/ttyACM* 2>/dev/null && echo "✓ Detected" || echo "✗ Not detected"
echo ""
echo -n "GNSS node: "
ros2 node list 2>/dev/null | grep -q gnss && echo "✓ Running" || echo "✗ Not running"
EOF
    chmod +x "$PROJECT_DIR/test_gnss.sh"
    echo "✓ Created test_gnss.sh"
fi

# ODrive test script
if [ ! -f "$PROJECT_DIR/test_odrive.sh" ]; then
    cat > "$PROJECT_DIR/test_odrive.sh" << 'EOF'
#!/bin/bash
# Quick test script for ODrive
echo "Testing ODrive..."
echo ""
if command -v odrivetool &>/dev/null; then
    echo "✓ odrivetool installed: $(odrivetool --version 2>&1 | head -1)"
else
    echo "✗ odrivetool not found in PATH"
fi
echo ""
echo "USB devices:"
lsusb | grep -i odrive || echo "  No ODrive detected"
EOF
    chmod +x "$PROJECT_DIR/test_odrive.sh"
    echo "✓ Created test_odrive.sh"
fi

# Create GNSS configuration script (if not already exists)
if [ ! -f "$PROJECT_DIR/configure_gnss_ubx.py" ]; then
    echo "⚠ configure_gnss_ubx.py not found - needs to be created manually"
fi

# Create documentation structure
echo ""
echo "Setting up documentation..."
mkdir -p "$PROJECT_DIR/docs"
mkdir -p "$PROJECT_DIR/logs"

echo ""
echo "✓ Hardware configuration complete"
echo ""
echo "Created utility scripts:"
echo "  • $PROJECT_DIR/test_dual_lidar.sh"
echo "  • $PROJECT_DIR/test_gnss.sh"
echo "  • $PROJECT_DIR/test_odrive.sh"
echo ""
echo "Network configuration:"
echo "  • LiDAR network: 192.168.2.100/24 on eno1"
echo "  • Front LiDAR: 192.168.2.62:6201 (configure manually)"
echo "  • Rear LiDAR: 192.168.2.63:6202 (configure manually)"
echo ""
echo "Next steps:"
echo "  1. Configure LiDARs via serial (IP addresses and target ports)"
echo "  2. Run: python3 ~/Tank_projects/configure_gnss_ubx.py (to configure GNSS)"
echo "  3. Test with utility scripts"

