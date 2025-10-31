#!/bin/bash
# CAN Bus Test Script for Jetson Orin Nano + ODrive
# Tests CAN interface, ODrive connectivity, and basic communication

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "=========================================="
echo "CAN Bus Test for ODrive + Jetson Orin Nano"
echo "=========================================="
echo ""

# Test 1: Check if CAN kernel modules are available
echo -e "${YELLOW}[1/8] Checking CAN kernel modules...${NC}"
if lsmod | grep -q "can"; then
    echo -e "${GREEN}✓ CAN modules loaded${NC}"
else
    echo -e "${YELLOW}⚠ Loading CAN modules...${NC}"
    sudo modprobe can
    sudo modprobe can_raw
    sudo modprobe mttcan
    if lsmod | grep -q "can"; then
        echo -e "${GREEN}✓ CAN modules loaded successfully${NC}"
    else
        echo -e "${RED}✗ Failed to load CAN modules${NC}"
        exit 1
    fi
fi
echo ""

# Test 2: Check if can0 interface exists
echo -e "${YELLOW}[2/8] Checking CAN interface...${NC}"
if ip link show can0 &> /dev/null; then
    echo -e "${GREEN}✓ can0 interface found${NC}"
else
    echo -e "${RED}✗ can0 interface not found${NC}"
    echo "Check if CAN is enabled in device tree"
    exit 1
fi
echo ""

# Test 3: Configure CAN interface
echo -e "${YELLOW}[3/8] Configuring CAN interface (500 kbps)...${NC}"
sudo ip link set down can0 2>/dev/null || true
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0

CAN_STATE=$(ip -details link show can0 | grep -oP '(?<=state )\w+')
if [ "$CAN_STATE" == "ERROR-ACTIVE" ]; then
    echo -e "${GREEN}✓ CAN interface configured: $CAN_STATE${NC}"
elif [ "$CAN_STATE" == "ERROR-WARNING" ]; then
    echo -e "${YELLOW}⚠ CAN interface state: $CAN_STATE (some errors, but usable)${NC}"
elif [ "$CAN_STATE" == "BUS-OFF" ]; then
    echo -e "${RED}✗ CAN interface state: BUS-OFF${NC}"
    echo "  Possible causes:"
    echo "  - No termination resistor (flip switch on CAN Pal)"
    echo "  - Wrong wiring (check CANH/CANL)"
    echo "  - No devices on bus"
    exit 1
else
    echo -e "${YELLOW}⚠ CAN interface state: $CAN_STATE${NC}"
fi
echo ""

# Test 4: Check if can-utils is installed
echo -e "${YELLOW}[4/8] Checking for can-utils...${NC}"
if command -v candump &> /dev/null; then
    echo -e "${GREEN}✓ can-utils installed${NC}"
else
    echo -e "${YELLOW}⚠ Installing can-utils...${NC}"
    sudo apt update && sudo apt install -y can-utils
    echo -e "${GREEN}✓ can-utils installed${NC}"
fi
echo ""

# Test 5: Send test CAN message (loopback test)
echo -e "${YELLOW}[5/8] Testing CAN loopback...${NC}"
echo "Sending test message: 0x123#DEADBEEF"

# Start candump in background
candump can0 > /tmp/can_test.log 2>&1 &
CANDUMP_PID=$!
sleep 0.5

# Send test message
cansend can0 123#DEADBEEF
sleep 0.5

# Check if message was received
kill $CANDUMP_PID 2>/dev/null
if grep -q "123" /tmp/can_test.log; then
    echo -e "${GREEN}✓ CAN loopback working (message sent and received)${NC}"
else
    echo -e "${RED}✗ CAN loopback failed${NC}"
    echo "Message was not received. Check interface configuration."
fi
rm -f /tmp/can_test.log
echo ""

# Test 6: Check for ODrive on CAN bus
echo -e "${YELLOW}[6/8] Checking for ODrive on CAN bus...${NC}"
echo "Listening for ODrive heartbeat messages (5 seconds)..."
echo "Expected: Periodic messages with IDs 0x001, 0x021, etc."
echo ""

timeout 5 candump can0 2>/dev/null | tee /tmp/odrive_test.log &
CANDUMP_PID=$!
sleep 5.5

if [ -s /tmp/odrive_test.log ]; then
    echo -e "${GREEN}✓ CAN traffic detected:${NC}"
    head -n 5 /tmp/odrive_test.log
    
    # Check for ODrive heartbeat pattern (0x001, 0x021, 0x041, etc.)
    if grep -qE "00[1248]|02[1248]|04[1248]|06[1248]" /tmp/odrive_test.log; then
        echo -e "${GREEN}✓ ODrive heartbeat messages detected!${NC}"
        echo "  ODrive is communicating on CAN bus"
    else
        echo -e "${YELLOW}⚠ CAN traffic detected, but no ODrive heartbeat pattern${NC}"
        echo "  Check ODrive CAN configuration:"
        echo "    odrv0.config.enable_can_a = True"
        echo "    odrv0.axis0.config.can_node_id = 0"
        echo "    odrv0.can.config.baud_rate = 500000"
    fi
else
    echo -e "${RED}✗ No CAN traffic detected${NC}"
    echo "  Possible causes:"
    echo "  - ODrive not connected"
    echo "  - ODrive CAN not enabled (use odrivetool to configure)"
    echo "  - Wrong wiring (CANH, CANL, GND)"
    echo "  - Missing termination resistor"
fi
rm -f /tmp/odrive_test.log
echo ""

# Test 7: Send velocity command to ODrive (if detected)
echo -e "${YELLOW}[7/8] Testing ODrive velocity command...${NC}"
echo "Attempting to send velocity command to axis 0"
echo "WARNING: Motor may move if enabled! Ensure safe setup."
read -p "Press Enter to send command, or Ctrl+C to skip..."

# Send velocity setpoint: 0.5 rad/s on axis 0
# CAN ID = 0x00B (axis 0), Data = velocity (float32) + torque_ff (float32)
python3 << 'EOF'
import struct
import os
import sys

try:
    vel = 0.5      # rad/s
    torque_ff = 0.0
    
    # Pack as little-endian floats
    data = struct.pack('<ff', vel, torque_ff)
    hex_data = data.hex().upper()
    
    # Split into bytes for cansend format
    formatted = ''.join([hex_data[i:i+2] for i in range(0, len(hex_data), 2)])
    
    cmd = f"cansend can0 00B#{formatted}"
    print(f"Sending: {cmd}")
    result = os.system(cmd)
    
    if result == 0:
        print("✓ Command sent successfully")
        sys.exit(0)
    else:
        print("✗ Failed to send command")
        sys.exit(1)
except Exception as e:
    print(f"✗ Error: {e}")
    sys.exit(1)
EOF

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Velocity command sent to ODrive${NC}"
    echo "  Check if motor responds (should rotate slowly if enabled)"
else
    echo -e "${RED}✗ Failed to send velocity command${NC}"
fi
echo ""

# Test 8: Summary and recommendations
echo -e "${YELLOW}[8/8] Test Summary${NC}"
echo "=========================================="

if ip -details link show can0 | grep -q "ERROR-ACTIVE"; then
    echo -e "${GREEN}✓ CAN interface operational${NC}"
    
    if [ -f /tmp/odrive_detected ]; then
        echo -e "${GREEN}✓ ODrive detected on CAN bus${NC}"
        echo ""
        echo "NEXT STEPS:"
        echo "1. Implement tank_odrive_can ROS2 package"
        echo "2. Use CAN for robust motor control"
        echo "3. Skip USB firmware compatibility issues"
    else
        echo -e "${YELLOW}⚠ ODrive not detected on CAN bus${NC}"
        echo ""
        echo "TO FIX:"
        echo "1. Connect ODrive to CAN Pal (CANH, CANL, GND)"
        echo "2. Enable 120Ω termination on CAN Pal (flip switch)"
        echo "3. Configure ODrive via USB:"
        echo "   odrivetool"
        echo "   odrv0.config.enable_can_a = True"
        echo "   odrv0.axis0.config.can_node_id = 0"
        echo "   odrv0.axis1.config.can_node_id = 1"
        echo "   odrv0.can.config.baud_rate = 500000"
        echo "   odrv0.save_configuration()"
        echo "   odrv0.reboot()"
        echo "4. Re-run this test script"
    fi
else
    echo -e "${RED}✗ CAN interface not operational${NC}"
    echo ""
    echo "TROUBLESHOOTING:"
    echo "1. Check Jetson CAN device tree configuration"
    echo "2. Verify CAN Pal connections to Jetson"
    echo "3. Check power to CAN Pal (3.3V)"
fi

echo ""
echo "For detailed implementation guide, see:"
echo "  workspace_structure.md - Section: CAN Bus vs USB for ODrive Control"
echo "=========================================="

