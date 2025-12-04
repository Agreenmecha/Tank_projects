#!/bin/bash
# Quick script to check ODrive tool version and ODrive firmware version

echo "=========================================="
echo "ODrive Version Checker"
echo "=========================================="
echo ""

# Check if odrivetool is installed
if ! command -v odrivetool &> /dev/null; then
    echo "❌ odrivetool is NOT installed"
    echo ""
    echo "Install with:"
    echo "  pip3 install odrive==0.5.4"
    exit 1
fi

# Check odrivetool version
echo "📦 ODrive Tool Version:"
odrivetool --version 2>&1 | head -n 1
echo ""

# Check pip package version
echo "📦 ODrive Python Package:"
pip3 list | grep odrive
echo ""

# Try to connect and get firmware version
echo "🔌 Attempting to connect to ODrive..."
python3 << 'EOF'
import odrive
import sys

try:
    odrv = odrive.find_any(timeout=5)
    if odrv:
        print(f"✅ Connected to ODrive: {odrv.serial_number}")
        print(f"   Hardware: v{odrv.hw_version_major}.{odrv.hw_version_minor}")
        print(f"   Firmware: v{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")
        print(f"   Bus Voltage: {odrv.vbus_voltage:.2f}V")
    else:
        print("❌ No ODrive found")
except Exception as e:
    print(f"❌ Error: {e}")
    sys.exit(1)
EOF

echo ""
echo "=========================================="
echo "Compatibility Check:"
echo "=========================================="
echo ""
echo "✅ RECOMMENDED: ODrive Tool v0.5.4 with Firmware v0.5.6"
echo "⚠️  Newer odrive tool versions may NOT work with FW v0.5.6"
echo ""
echo "If you have version mismatch:"
echo "  pip3 uninstall odrive"
echo "  pip3 install odrive==0.5.4"
echo ""

