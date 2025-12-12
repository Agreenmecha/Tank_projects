#!/bin/bash
################################################################################
# Fix ODrive USB Permissions - PERMANENT
# Run this script with: sudo ./fix_odrive_permissions.sh
################################################################################

echo "=========================================="
echo "Fixing ODrive USB Permissions (PERMANENT)"
echo "=========================================="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "❌ Please run with sudo"
    exit 1
fi

# Create comprehensive udev rule
echo "1. Creating udev rule..."
cat > /etc/udev/rules.d/50-odrive.rules << 'EOF'
# ODrive USB Permissions
# Matches all ODrive USB devices (Product ID 0d00-0d99)
SUBSYSTEM=="usb", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="0d[0-9][0-9]", MODE="0666", GROUP="dialout"
SUBSYSTEM=="usb", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="0d32", MODE="0666", GROUP="dialout"
# DFU mode for firmware updates
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="df11", MODE="0666", GROUP="dialout"
EOF
echo "✓ Created /etc/udev/rules.d/50-odrive.rules"
cat /etc/udev/rules.d/50-odrive.rules
echo ""

# Add user to dialout group
echo "2. Adding user to dialout group..."
usermod -a -G dialout $SUDO_USER
echo "✓ User $SUDO_USER added to dialout group"
echo ""

# Reload udev rules
echo "3. Reloading udev rules..."
udevadm control --reload-rules
udevadm trigger --attr-match=idVendor=1209
echo "✓ Udev rules reloaded and triggered"
echo ""

# Verify the rule is loaded
echo "4. Verifying udev rule..."
if udevadm control --reload-rules 2>&1 | grep -q "error"; then
    echo "⚠️  Warning: udev reload reported an error"
else
    echo "✓ Udev rules loaded successfully"
fi
echo ""

echo "=========================================="
echo "✓ PERMANENT FIX COMPLETE!"
echo "=========================================="
echo ""
echo "The fix will persist across reboots."
echo ""
echo "Next steps:"
echo "  1. Unplug and replug the ODrive USB cable"
echo "  2. Check permissions: ls -l /dev/bus/usb/001/005"
echo "     (Should show: crw-rw-rw-)"
echo "  3. Test: ./drive.sh"
echo ""

