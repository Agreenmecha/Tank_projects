#!/bin/bash
# e-CAM25_CUONX Installation for JetPack 6.2.1 (L4T 36.4.7)
# Quick install using JetPack 6.1 pre-built binaries (usually compatible)

set -e  # Exit on error

echo "============================================="
echo " e-CAM25_CUONX Installation for JetPack 6.2.1"
echo " Date: $(date)"
echo "============================================="

# Check if running as root
if [[ $EUID -ne 0 ]]; then
   echo "Please run as root: sudo ./install_ecam25_jp621.sh"
   exit 1
fi

# Verify L4T version
echo ""
echo "Checking L4T version..."
cat /etc/nv_tegra_release
echo ""

# Set paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RELEASE_DIR="$SCRIPT_DIR/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_29-MAR-2025_R04/e-CAM25_CUONX_L4T36.4.0_JP6.1.0_JETSON-ONX-ONANO_R04/ONX_ONANO"

if [ ! -d "$RELEASE_DIR" ]; then
    echo "ERROR: Release directory not found: $RELEASE_DIR"
    echo "Please extract the release package first"
    exit 1
fi

# Prompt for lane configuration
echo "Select lane configuration:"
echo "  1) 2-lane (up to 30fps @ 1920x1200)"
echo "  2) 4-lane (up to 70fps @ 1920x1080) [RECOMMENDED]"
read -p "Enter choice [1 or 2]: " lane_choice

if [[ "$lane_choice" == "1" ]]; then
    LANE_MODE="2lane"
    DTBO_FILE="tegra234-p3767-0000-p3768-0000-a0-2lane-ar0234.dtbo"
elif [[ "$lane_choice" == "2" ]]; then
    LANE_MODE="4lane"
    DTBO_FILE="tegra234-p3767-0000-p3768-0000-a0-4lane-ar0234.dtbo"
else
    echo "Invalid choice"
    exit 1
fi

echo "Using ${LANE_MODE} configuration"
echo ""

# Step 1: Install kernel modules
echo "============================================="
echo "Step 1: Installing kernel modules..."
echo "============================================="

# Create directories if they don't exist
mkdir -p /lib/modules/$(uname -r)/kernel/drivers/media/i2c/
mkdir -p /lib/modules/$(uname -r)/kernel/drivers/media/platform/tegra/camera/
mkdir -p /lib/modules/$(uname -r)/kernel/drivers/video/tegra/host/

cp -v "$RELEASE_DIR/Kernel/ar0234.ko" /lib/modules/$(uname -r)/kernel/drivers/media/i2c/
cp -v "$RELEASE_DIR/Kernel/tegra-camera.ko" /lib/modules/$(uname -r)/kernel/drivers/media/platform/tegra/camera/
cp -v "$RELEASE_DIR/Kernel/capture-ivc.ko" /lib/modules/$(uname -r)/kernel/drivers/video/tegra/host/

# Update module dependencies
echo "Updating module dependencies..."
depmod -a

# Step 2: Install DTBO
echo ""
echo "============================================="
echo "Step 2: Installing Device Tree Overlay..."
echo "============================================="
cp -v "$RELEASE_DIR/Kernel/$DTBO_FILE" /boot/
echo "DTBO installed: /boot/$DTBO_FILE"

# Step 3: Load overlay (manual configuration)
echo ""
echo "============================================="
echo "Step 3: Configuring device tree overlay..."
echo "============================================="

# Manually add DTBO to extlinux.conf
EXTLINUX_CONF="/boot/extlinux/extlinux.conf"

echo "Backing up extlinux.conf..."
cp "$EXTLINUX_CONF" "${EXTLINUX_CONF}.backup_$(date +%Y%m%d_%H%M%S)"

# Check if DTBO already exists in config
if grep -q "$DTBO_FILE" "$EXTLINUX_CONF"; then
    echo "DTBO already configured in $EXTLINUX_CONF"
else
    echo "Adding DTBO to $EXTLINUX_CONF..."
    # Add FDT line after the LINUX line in the primary boot entry
    sed -i "/LINUX \/boot\/Image/a\      FDT /boot/dtb/kernel_tegra234-p3767-0000-p3768-0000-a0.dtb\n      OVERLAYS /boot/$DTBO_FILE" "$EXTLINUX_CONF"
    echo "DTBO configured: /boot/$DTBO_FILE"
fi

echo ""
echo "============================================="
echo " Installation Complete!"
echo "============================================="
echo ""
echo "The system will REBOOT in 5 seconds to activate the camera."
echo "After reboot, test with:"
echo "  ls /dev/video*                      # Should show /dev/video0"
echo "  v4l2-ctl --list-devices             # List camera details"
echo "  v4l2-ctl --all --device /dev/video0 # Camera capabilities"
echo ""
echo "Press Ctrl+C to cancel reboot..."
sleep 5

echo "Rebooting now..."
reboot

