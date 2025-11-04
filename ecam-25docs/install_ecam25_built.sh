#!/bin/bash
################################################################################
# e-CAM25_CUONX Driver Installation Script (for Jetson)
# Install drivers built on Linux PC
################################################################################

set -e

if [[ $EUID -ne 0 ]]; then
   echo "Please run as root: sudo ./install_ecam25_built.sh"
   exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOME_DIR=$(eval echo ~$SUDO_USER)

echo "=========================================="
echo " e-CAM25_CUONX Driver Installation"
echo " Date: $(date)"
echo "=========================================="
echo ""

# Check for DTBO file
DTBO_2LANE="$HOME_DIR/tegra234-p3767-0000-p3768-0000-a0-2lane-ar0234.dtbo"
DTBO_4LANE="$HOME_DIR/tegra234-p3767-0000-p3768-0000-a0-4lane-ar0234.dtbo"

if [ -f "$DTBO_4LANE" ]; then
    DTBO_FILE="$DTBO_4LANE"
    LANE_MODE="4lane"
elif [ -f "$DTBO_2LANE" ]; then
    DTBO_FILE="$DTBO_2LANE"
    LANE_MODE="2lane"
else
    echo "ERROR: DTBO file not found in home directory!"
    echo "Expected: $DTBO_4LANE OR $DTBO_2LANE"
    echo ""
    echo "Please copy DTBO file from Linux PC to Jetson home directory:"
    echo "  scp tegra234-p3767-0000-p3768-0000-a0-4lane-ar0234.dtbo aaron@<jetson-ip>:~/"
    exit 1
fi

echo "Using ${LANE_MODE} configuration: $(basename $DTBO_FILE)"
echo ""

# Check for kernel modules
MODULES_TAR="$HOME_DIR/kernel_supplements.tar.bz2"
if [ ! -f "$MODULES_TAR" ]; then
    echo "ERROR: Kernel modules archive not found: $MODULES_TAR"
    echo ""
    echo "Please copy from Linux PC:"
    echo "  scp kernel_supplements.tar.bz2 aaron@<jetson-ip>:~/"
    exit 1
fi

# Step 1: Install DTBO
echo "=========================================="
echo "Step 1: Installing Device Tree Overlay"
echo "=========================================="
echo ""

# Backup extlinux.conf
EXTLINUX_CONF="/boot/extlinux/extlinux.conf"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
cp "$EXTLINUX_CONF" "${EXTLINUX_CONF}.backup_$TIMESTAMP"
echo "✓ Backed up extlinux.conf: ${EXTLINUX_CONF}.backup_$TIMESTAMP"

# Copy DTBO to /boot
cp "$DTBO_FILE" /boot/
echo "✓ Installed DTBO: /boot/$(basename $DTBO_FILE)"

# Add DTBO to extlinux.conf
if ! grep -q "$(basename $DTBO_FILE)" "$EXTLINUX_CONF"; then
    # Find the FDT line and add OVERLAYS after it
    sed -i "/FDT \/boot\/dtb\/kernel_tegra234-p3767-0000-p3768-0000-a0.dtb/a\      OVERLAYS /boot/$(basename $DTBO_FILE)" "$EXTLINUX_CONF"
    echo "✓ Added DTBO to extlinux.conf"
else
    echo "⚠ DTBO already in extlinux.conf"
fi

# Step 2: Install kernel modules
echo ""
echo "=========================================="
echo "Step 2: Installing Kernel Modules"
echo "=========================================="
echo ""

echo "Extracting kernel modules..."
tar -xjmpf "$MODULES_TAR" -C /usr/lib/
echo "✓ Kernel modules extracted"

echo ""
echo "Updating module dependencies..."
depmod -a
echo "✓ Module dependencies updated"

# Step 3: Install ISP tuning file (optional)
echo ""
echo "=========================================="
echo "Step 3: Installing ISP Tuning File"
echo "=========================================="
echo ""

ISP_FILE="$HOME_DIR/camera_overrides_jetson-onano.isp"
if [ -f "$ISP_FILE" ]; then
    mkdir -p /var/nvidia/nvcam/settings
    cp "$ISP_FILE" /var/nvidia/nvcam/settings/camera_overrides.isp
    chmod 664 /var/nvidia/nvcam/settings/camera_overrides.isp
    chown root:root /var/nvidia/nvcam/settings/camera_overrides.isp
    echo "✓ ISP tuning file installed"
else
    echo "⚠ ISP tuning file not found (optional, skipping)"
fi

# Summary
echo ""
echo "=========================================="
echo " Installation Complete!"
echo "=========================================="
echo ""
echo "Configuration: ${LANE_MODE}"
echo "DTBO: /boot/$(basename $DTBO_FILE)"
echo "Backup: ${EXTLINUX_CONF}.backup_$TIMESTAMP"
echo ""
echo "IMPORTANT: Reboot the system for changes to take effect"
echo ""
read -p "Reboot now? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "Rebooting..."
    reboot
else
    echo ""
    echo "Please reboot manually: sudo reboot"
    echo ""
    echo "After reboot, verify camera with:"
    echo "  ls /dev/video*"
    echo "  lsmod | grep ar0234"
    echo "  v4l2-ctl --list-devices"
fi

