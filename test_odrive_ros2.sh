#!/bin/bash
################################################################################
# Test ODrive ROS 2 Node
# Verify ODrive is connected and ROS 2 node can communicate
################################################################################

echo "=========================================="
echo "ODrive ROS 2 Node Test"
echo "=========================================="
echo ""

# Check if ODrive is connected
echo "1. Checking ODrive connection..."
if lsusb | grep -q "ODrive"; then
    echo -e "\033[0;32m✓\033[0m ODrive detected via USB"
    lsusb | grep ODrive
else
    echo -e "\033[0;31m❌\033[0m ODrive NOT detected via USB"
    echo "   Please connect ODrive via USB"
    exit 1
fi
echo ""

# Check odrivetool
echo "2. Checking odrivetool..."
if command -v odrivetool &>/dev/null; then
    ODRIVE_VERSION=$(odrivetool --version 2>&1 | head -1)
    echo -e "\033[0;32m✓\033[0m odrivetool installed: $ODRIVE_VERSION"
else
    echo -e "\033[0;31m❌\033[0m odrivetool not found"
    echo "   Install with: pip3 install odrive==0.5.4"
    exit 1
fi
echo ""

# Test direct connection
echo "3. Testing direct ODrive connection..."
python3 << 'EOF'
import odrive
from odrive.enums import *
import sys

try:
    print("   Searching for ODrive...")
    odrv = odrive.find_any(timeout=5)
    if odrv:
        print(f"   ✓ Connected to ODrive: {odrv.serial_number}")
        print(f"   ✓ Firmware: v{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")
        print(f"   ✓ Hardware: v{odrv.hw_version_major}.{odrv.hw_version_minor}")
        
        # Check axis states
        print(f"   ✓ Axis 0 state: {odrv.axis0.current_state}")
        print(f"   ✓ Axis 1 state: {odrv.axis1.current_state}")
        sys.exit(0)
    else:
        print("   ❌ ODrive not found")
        sys.exit(1)
except Exception as e:
    print(f"   ❌ Error: {e}")
    sys.exit(1)
EOF

if [ $? -eq 0 ]; then
    echo -e "\033[0;32m✓\033[0m Direct connection successful"
else
    echo -e "\033[0;31m❌\033[0m Direct connection failed"
    exit 1
fi
echo ""

# Check ROS 2 workspace
echo "4. Checking ROS 2 workspace..."
cd ~/Tank_projects/tank_ws
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo -e "\033[0;32m✓\033[0m Workspace sourced"
else
    echo -e "\033[0;31m❌\033[0m Workspace not built"
    echo "   Build with: colcon build --packages-select tank_control"
    exit 1
fi
echo ""

# Check if node exists
echo "5. Checking ROS 2 node..."
if ros2 pkg list | grep -q tank_control; then
    echo -e "\033[0;32m✓\033[0m tank_control package found"
    
    # Check if executable exists
    if ros2 run tank_control odrive_interface_node --help &>/dev/null; then
        echo -e "\033[0;32m✓\033[0m odrive_interface_node executable available"
    else
        echo -e "\033[0;33m⚠\033[0m Node executable not found (may need to rebuild)"
    fi
else
    echo -e "\033[0;31m❌\033[0m tank_control package not found"
    exit 1
fi
echo ""

# Check launch file
echo "6. Checking launch file..."
LAUNCH_FILE="src/tank_control/launch/odrive_interface.launch.py"
if [ -f "$LAUNCH_FILE" ]; then
    echo -e "\033[0;32m✓\033[0m Launch file found: $LAUNCH_FILE"
else
    echo -e "\033[0;31m❌\033[0m Launch file not found"
    exit 1
fi
echo ""

# Check config file
echo "7. Checking configuration..."
CONFIG_FILE="src/tank_control/config/odrive_params.yaml"
if [ -f "$CONFIG_FILE" ]; then
    echo -e "\033[0;32m✓\033[0m Config file found: $CONFIG_FILE"
    echo ""
    echo "   Current configuration:"
    grep -E "wheel_radius|track_width|max_vel|watchdog_timeout" "$CONFIG_FILE" | sed 's/^/      /'
else
    echo -e "\033[0;31m❌\033[0m Config file not found"
    exit 1
fi
echo ""

echo "=========================================="
echo -e "\033[0;32m✓ All checks passed!\033[0m"
echo "=========================================="
echo ""
echo "Ready to launch ODrive ROS 2 node:"
echo ""
echo "  cd ~/Tank_projects/tank_ws"
echo "  source install/setup.bash"
echo "  ros2 launch tank_control odrive_interface.launch.py"
echo ""
echo "Or test with:"
echo "  ros2 run tank_control odrive_interface_node"
echo ""

