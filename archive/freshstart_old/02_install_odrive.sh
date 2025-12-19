#!/bin/bash
################################################################################
# Install ODrive 0.5.4
# Motor controller software and tools
################################################################################

set -e

echo "Installing ODrive 0.5.4..."
echo ""

# Install ODrive Python package (version 0.5.4)
echo "Installing ODrive Python package..."
pip3 install --user odrive==0.5.4

# Add udev rules for ODrive USB access
echo ""
echo "Adding udev rules for ODrive..."
sudo bash -c 'cat > /etc/udev/rules.d/91-odrive.rules << EOF
# ODrive USB device rules
# Allows users in plugdev group to access ODrive
SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0d3[2-3]", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="df11", MODE="0666", GROUP="plugdev"
EOF'

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Add user to dialout and plugdev groups (for serial/USB access)
echo ""
echo "Adding user to required groups..."
sudo usermod -a -G dialout,plugdev $USER

# Ensure .local/bin is in PATH (should be done in step 01, but verify)
if ! grep -q '.local/bin' ~/.bashrc; then
    echo 'export PATH=$PATH:~/.local/bin' >> ~/.bashrc
fi

# Verify installation
echo ""
echo "Verifying ODrive installation..."
if command -v odrivetool &> /dev/null; then
    ODRIVE_VERSION=$(odrivetool --version 2>&1 | grep -oP 'ODrive control utility v\K[0-9.]+' || echo "unknown")
    echo "✓ odrivetool installed: version $ODRIVE_VERSION"
else
    # May not be in PATH yet
    if [ -f ~/.local/bin/odrivetool ]; then
        echo "✓ odrivetool installed at ~/.local/bin/odrivetool"
        echo "  (Will be available after terminal restart)"
    else
        echo "⚠ odrivetool not found, but ODrive package is installed"
    fi
fi

echo ""
echo "✓ ODrive 0.5.4 installed successfully"
echo ""
echo "Note: You may need to log out and log back in for group changes to take effect"
echo "      Or run: newgrp dialout"

