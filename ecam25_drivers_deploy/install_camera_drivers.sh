#!/bin/bash
# e-CAM25_CUONX Driver Installation Script for JetPack 6.2.1
# Built from L4T 36.4.3 sources (compatible with 36.4.7)
# 
# Usage: sudo ./install_camera_drivers.sh [2lane|4lane]

set -e

if [[ $EUID -ne 0 ]]; then
   echo "Please run as root: sudo $0"
   exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LANE_CONFIG="${1:-4lane}"

if [[ "$LANE_CONFIG" != "2lane" && "$LANE_CONFIG" != "4lane" ]]; then
    echo "Usage: $0 [2lane|4lane]"
    echo "  2lane: Up to 30fps @ 1920x1200"
    echo "  4lane: Up to 70fps @ 1920x1080 [RECOMMENDED]"
    exit 1
fi

echo "=========================================="
echo " e-CAM25_CUONX Driver Installation"
echo " Configuration: $LANE_CONFIG"
echo " Date: $(date)"
echo "=========================================="
echo

# Backup extlinux.conf
echo "1. Backing up extlinux.conf..."
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
cp /boot/extlinux/extlinux.conf /boot/extlinux/extlinux.conf.backup_$TIMESTAMP
echo "   ✓ Backup created: extlinux.conf.backup_$TIMESTAMP"

# Install DTBO
echo "2. Installing device tree overlay..."
DTBO_FILE="tegra234-p3767-0000-p3768-0000-a0-${LANE_CONFIG}-ar0234.dtbo"
if [ ! -f "$SCRIPT_DIR/$DTBO_FILE" ]; then
    echo "ERROR: $DTBO_FILE not found in $SCRIPT_DIR"
    exit 1
fi

cp "$SCRIPT_DIR/$DTBO_FILE" /boot/
echo "   ✓ Installed: $DTBO_FILE"

# Update extlinux.conf
echo "3. Updating extlinux.conf..."
if ! grep -q "FDT /boot/$DTBO_FILE" /boot/extlinux/extlinux.conf; then
    sed -i "/FDT \/boot\/dtb\/kernel_tegra234-p3768-0000+p3767-0000-nv.dtb/a\      FDT /boot/$DTBO_FILE" /boot/extlinux/extlinux.conf
    echo "   ✓ Added FDT overlay entry"
else
    echo "   ✓ FDT overlay already present"
fi

echo
echo "=========================================="
echo " Installation Complete!"
echo "=========================================="
echo
echo "Configuration: $LANE_CONFIG"
echo "DTBO: $DTBO_FILE"
echo "Backup: /boot/extlinux/extlinux.conf.backup_$TIMESTAMP"
echo
echo "IMPORTANT: Reboot the system for changes to take effect"
echo "  sudo reboot"
echo
echo "After reboot, verify camera with:"
echo "  ls /dev/video*"
echo "  v4l2-ctl --list-devices"
echo "  gst-launch-1.0 v4l2src device=/dev/video0 ! xvimagesink"
echo
