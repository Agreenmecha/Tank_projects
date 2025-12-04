#!/bin/bash
################################################################################
# Fix ODrive USB Permissions
# Run this script with: sudo ./fix_odrive_permissions.sh
################################################################################

echo "=========================================="
echo "Fixing ODrive USB Permissions"
echo "=========================================="
echo ""

# Create udev rule
echo "1. Creating udev rule..."
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0d[0-9][0-9]", MODE="0666"' > /etc/udev/rules.d/50-odrive.rules
echo "✓ Created /etc/udev/rules.d/50-odrive.rules"
echo ""

# Reload udev rules
echo "2. Reloading udev rules..."
udevadm control --reload-rules
udevadm trigger
echo "✓ Udev rules reloaded"
echo ""

# Add user to dialout group
echo "3. Adding user to dialout group..."
usermod -a -G dialout $SUDO_USER
echo "✓ User $SUDO_USER added to dialout group"
echo ""

echo "=========================================="
echo "✓ Setup complete!"
echo "=========================================="
echo ""
echo "IMPORTANT: You need to log out and back in for group changes to take effect."
echo "Or run: newgrp dialout"
echo ""

