#!/bin/bash
################################################################################
# e-CAM25_CUONX Driver Build Script for JetPack 6.2.1
# Build on Native Linux PC → Install on Jetson Orin Nano
# Repository: Tank_projects
################################################################################

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}"
echo "================================================================================"
echo "  e-CAM25_CUONX Driver Build for JetPack 6.2.1"
echo "  Building camera drivers for Jetson Orin Nano"
echo "  Repository: Tank_projects"
echo "================================================================================"
echo -e "${NC}"
echo ""

# Check if running on x86_64 (not aarch64/Jetson)
ARCH=$(uname -m)
if [ "$ARCH" == "aarch64" ]; then
    echo -e "${RED}ERROR: This script must run on a Linux PC (x86_64), not on Jetson (aarch64)!${NC}"
    echo "Please run this on your native Linux PC."
    exit 1
fi

# Detect WSL
if grep -qi microsoft /proc/version; then
    echo -e "${RED}ERROR: This script cannot run in WSL!${NC}"
    echo "NVIDIA kernel module compilation requires native Linux kernel headers."
    echo "Please run this on a native Linux PC (Ubuntu 20.04/22.04)."
    exit 1
fi

echo -e "${GREEN}✓ Running on native Linux PC (${ARCH})${NC}"
echo ""

# Get the repository root (assuming script is in Tank_projects/camera_build_workspace/scripts/)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
WORKSPACE="$REPO_ROOT/camera_build_workspace"

echo "Repository root: $REPO_ROOT"
echo "Workspace: $WORKSPACE"
echo ""

# Set environment variables
export TOP_DIR="$WORKSPACE"
export RELEASE_PACK_DIR="$REPO_ROOT/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_29-MAR-2025_R04"
export NVIDIA_FILES_DIR="$REPO_ROOT/nvidiaR36.4.3"
export NVIDIA_SRC="$WORKSPACE/Linux_for_Tegra/source"
export LDK_ROOTFS_DIR="$WORKSPACE/Linux_for_Tegra/rootfs"
export MAKEFILE_DIR="$WORKSPACE/sensor_driver"
export CROSS_COMPILE="$WORKSPACE/aarch64--glibc--stable-2022.08-1/bin/aarch64-buildroot-linux-gnu-"
export KERNEL_HEADERS="$NVIDIA_SRC/kernel/kernel-jammy-src"
export NVIDIA_HEADERS="$NVIDIA_SRC/nvidia-oot"
export NVIDIA_CONFTEST="$NVIDIA_HEADERS/conftest"
export OUTPUT_DIR="$WORKSPACE/jetson_binaries"

echo -e "${GREEN}Environment configured${NC}"
echo ""

# Verify all required files exist
echo "Verifying required files..."
REQUIRED_FILES=(
    "$WORKSPACE/aarch64--glibc--stable-2022.08-1.tar.bz2"
    "$NVIDIA_FILES_DIR/Jetson_Linux_R36.4.3_aarch64.tbz2"
    "$NVIDIA_FILES_DIR/Tegra_Linux_Sample-Root-Filesystem_R36.4.3_aarch64.tbz2"
    "$NVIDIA_FILES_DIR/public_sources.tbz2"
)

MISSING_FILES=0
for file in "${REQUIRED_FILES[@]}"; do
    if [ ! -f "$file" ]; then
        echo -e "${RED}✗ Missing: $file${NC}"
        MISSING_FILES=$((MISSING_FILES + 1))
    else
        echo -e "${GREEN}✓ Found: $(basename "$file")${NC}"
    fi
done

if [ ! -d "$RELEASE_PACK_DIR" ]; then
    echo -e "${RED}✗ Missing: e-CAM25 release package${NC}"
    echo "Expected: $RELEASE_PACK_DIR"
    MISSING_FILES=$((MISSING_FILES + 1))
else
    echo -e "${GREEN}✓ Found: e-CAM25 release package${NC}"
fi

if [ $MISSING_FILES -gt 0 ]; then
    echo ""
    echo -e "${RED}ERROR: $MISSING_FILES required file(s) missing!${NC}"
    echo ""
    echo "Please download missing files:"
    echo "  • Bootlin toolchain: https://toolchains.bootlin.com/downloads/releases/toolchains/aarch64/tarballs/"
    echo "  • NVIDIA files: https://developer.nvidia.com/embedded/jetpack (L4T 36.4.3)"
    echo ""
    echo "See docs/BUILD_GUIDE_NATIVE_PC.md for complete instructions."
    exit 1
fi

echo -e "${GREEN}✓ All required files present${NC}"
echo ""

# Step 1: Extract Bootlin toolchain
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}Step 1: Extracting Bootlin toolchain${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

if [ ! -d "$WORKSPACE/aarch64--glibc--stable-2022.08-1" ]; then
    echo "Extracting Bootlin toolchain..."
    cd "$WORKSPACE"
    tar -xf "$WORKSPACE/aarch64--glibc--stable-2022.08-1.tar.bz2"
    echo -e "${GREEN}✓ Toolchain extracted${NC}"
else
    echo -e "${GREEN}✓ Toolchain already extracted${NC}"
fi

# Step 2: Extract NVIDIA L4T packages
echo ""
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}Step 2: Extracting NVIDIA L4T packages${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

if [ ! -d "$WORKSPACE/Linux_for_Tegra" ]; then
    echo "Extracting driver package (5-10 minutes)..."
    cd "$WORKSPACE"
    sudo tar -xpf "$NVIDIA_FILES_DIR/Jetson_Linux_R36.4.3_aarch64.tbz2"
    echo -e "${GREEN}✓ Driver package extracted${NC}"
    
    echo ""
    echo "Extracting root filesystem (10-15 minutes)..."
    cd "$WORKSPACE/Linux_for_Tegra/rootfs"
    sudo tar -xpf "$NVIDIA_FILES_DIR/Tegra_Linux_Sample-Root-Filesystem_R36.4.3_aarch64.tbz2"
    echo -e "${GREEN}✓ Root filesystem extracted${NC}"
    
    echo ""
    echo "Applying NVIDIA binaries..."
    cd "$WORKSPACE/Linux_for_Tegra"
    sudo ./apply_binaries.sh
    echo -e "${GREEN}✓ Binaries applied${NC}"
else
    echo -e "${GREEN}✓ L4T packages already extracted${NC}"
fi

# Step 3: Extract kernel sources
echo ""
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}Step 3: Extracting kernel sources${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

if [ ! -d "$NVIDIA_SRC/kernel/kernel-jammy-src" ]; then
    echo "Extracting public sources (5-10 minutes)..."
    cd "$WORKSPACE"
    tar -xf "$NVIDIA_FILES_DIR/public_sources.tbz2"
    
    cd "$NVIDIA_SRC"
    echo "Extracting kernel sources..."
    tar -xf kernel_src.tbz2
    echo "Extracting OOT modules..."
    tar -xf kernel_oot_modules_src.tbz2
    echo -e "${GREEN}✓ Kernel sources extracted${NC}"
else
    echo -e "${GREEN}✓ Kernel sources already extracted${NC}"
fi

# Step 4: Apply e-CAM25 kernel patches
echo ""
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}Step 4: Applying e-CAM25 kernel patches${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

cd "$KERNEL_HEADERS"

# Find patch files (look in both possible locations)
if [ -d "$RELEASE_PACK_DIR/Kernel" ]; then
    PATCH_DIR="$RELEASE_PACK_DIR/Kernel"
elif [ -d "$RELEASE_PACK_DIR/e-CAM25_CUONX_L4T36.4.0_JP6.1.0_JETSON-ONX-ONANO_R04/ONX_ONANO/Kernel" ]; then
    PATCH_DIR="$RELEASE_PACK_DIR/e-CAM25_CUONX_L4T36.4.0_JP6.1.0_JETSON-ONX-ONANO_R04/ONX_ONANO/Kernel"
else
    echo -e "${RED}ERROR: Cannot find patch directory in e-CAM25 release package${NC}"
    exit 1
fi

MODULE_PATCH="$PATCH_DIR/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_module.patch"
DTB_PATCH="$PATCH_DIR/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_dtb.patch"
OOT_PATCH="$PATCH_DIR/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_oot.patch"

# 4.1: Apply module patch
echo "4.1: Applying module patch..."
mkdir -p "$MAKEFILE_DIR"
if [ ! -f "$MAKEFILE_DIR/.module_patch_applied" ]; then
    patch -d "$MAKEFILE_DIR" -p1 -i "$MODULE_PATCH" && touch "$MAKEFILE_DIR/.module_patch_applied"
    echo -e "${GREEN}✓ Module patch applied${NC}"
else
    echo -e "${YELLOW}⚠ Module patch already applied${NC}"
fi

# 4.2: Modify and apply device tree patch
echo ""
echo "4.2: Modifying and applying device tree patch..."
if [ ! -f "$DTB_PATCH.modified" ]; then
    cp "$DTB_PATCH" "$DTB_PATCH.backup"
    sed -i 's/vcc-supply/\/\/vcc-supply/g' "$DTB_PATCH"
    touch "$DTB_PATCH.modified"
    echo -e "${GREEN}✓ Modified DTB patch for 6.2.x${NC}"
fi

if [ ! -f "$KERNEL_HEADERS/.dtb_patch_applied" ]; then
    patch -p1 -i "$DTB_PATCH" --forward || echo -e "${YELLOW}⚠ DTB patch may have been partially applied (normal)${NC}"
    touch "$KERNEL_HEADERS/.dtb_patch_applied"
    echo -e "${GREEN}✓ Device tree patch applied${NC}"
else
    echo -e "${YELLOW}⚠ Device tree patch already applied${NC}"
fi

# 4.3: Modify and apply nvidia-oot patch
echo ""
echo "4.3: Modifying and applying nvidia-oot patch..."
if [ ! -f "$OOT_PATCH.modified" ]; then
    cp "$OOT_PATCH" "$OOT_PATCH.backup"
    sed '/@@ -2281,6 +2285,28 @@ static long tegra_channel_default_ioctl/,/tegra_channel_close/d' "$OOT_PATCH" > /tmp/tempfile && \
        mv /tmp/tempfile "$OOT_PATCH"
    touch "$OOT_PATCH.modified"
    echo -e "${GREEN}✓ Modified OOT patch for 6.2.x${NC}"
fi

if [ ! -f "$KERNEL_HEADERS/.oot_patch_applied" ]; then
    patch -p1 -i "$OOT_PATCH" --forward || echo -e "${YELLOW}⚠ OOT patch may have been partially applied${NC}"
    touch "$KERNEL_HEADERS/.oot_patch_applied"
    echo -e "${GREEN}✓ nvidia-oot patch applied${NC}"
else
    echo -e "${YELLOW}⚠ nvidia-oot patch already applied${NC}"
fi

# Step 5: Build kernel and modules
echo ""
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}Step 5: Building kernel and modules${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo -e "${YELLOW}This step takes 30-60 minutes depending on your PC...${NC}"
echo ""

cd "$NVIDIA_SRC"

# Build kernel
echo "Building kernel..."
make -C kernel -j$(nproc)
echo -e "${GREEN}✓ Kernel built${NC}"

# Build modules
echo ""
echo "Building modules (this takes the longest)..."
make modules -j$(nproc)
echo -e "${GREEN}✓ Modules built${NC}"

# Install modules to rootfs
echo ""
echo "Installing modules to rootfs..."
sudo -E make modules_install
echo -e "${GREEN}✓ Modules installed${NC}"

# Build device tree blobs
echo ""
echo "Building device tree blobs..."
make dtbs -j$(nproc)
echo -e "${GREEN}✓ Device tree blobs built${NC}"

# Step 6: Package binaries for Jetson
echo ""
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}Step 6: Packaging binaries for Jetson${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

# Create output directory
mkdir -p "$OUTPUT_DIR"

# Use 4-lane configuration (recommended for Tank project)
LANE_MODE="4lane"
DTBO_FILE="tegra234-p3767-0000-p3768-0000-a0-4lane-ar0234.dtbo"

echo "Using ${LANE_MODE} configuration (1280x720 @ 20fps)"
echo ""

# Copy DTBO
DTBO_SOURCE="$NVIDIA_SRC/kernel-devicetree/generic-dts/dtbs/$DTBO_FILE"
if [ -f "$DTBO_SOURCE" ]; then
    cp "$DTBO_SOURCE" "$OUTPUT_DIR/"
    echo -e "${GREEN}✓ DTBO copied: $DTBO_FILE${NC}"
else
    echo -e "${RED}ERROR: DTBO not found at $DTBO_SOURCE${NC}"
    exit 1
fi

# Compress kernel modules
echo ""
echo "Compressing kernel modules..."
cd "$LDK_ROOTFS_DIR/lib/"
sudo tar -cjf "$OUTPUT_DIR/kernel_supplements.tar.bz2" modules/
echo -e "${GREEN}✓ Kernel modules compressed${NC}"

# Copy ISP tuning file
ISP_FILE=$(find "$RELEASE_PACK_DIR" -name "camera_overrides*.isp" 2>/dev/null | head -1)
if [ -n "$ISP_FILE" ] && [ -f "$ISP_FILE" ]; then
    cp "$ISP_FILE" "$OUTPUT_DIR/camera_overrides_jetson-onano.isp"
    echo -e "${GREEN}✓ ISP tuning file copied${NC}"
else
    echo -e "${YELLOW}⚠ ISP tuning file not found (optional)${NC}"
fi

# Create installation script for Jetson
cat > "$OUTPUT_DIR/install_on_jetson.sh" << 'INSTALL_EOF'
#!/bin/bash
# Installation script for Jetson Orin Nano
# Run this ON THE JETSON after copying the binaries

set -e

echo "=============================================="
echo "  e-CAM25_CUONX Driver Installation"
echo "  For Jetson Orin Nano JetPack 6.2.1"
echo "=============================================="
echo ""

# Check if running on Jetson
if ! uname -r | grep -q "tegra"; then
    echo "ERROR: This script must run on the Jetson, not on PC!"
    exit 1
fi

# Install DTBO
echo "Installing device tree overlay..."
sudo cp tegra234-p3767-0000-p3768-0000-a0-4lane-ar0234.dtbo /boot/
echo "✓ DTBO installed"

# Install kernel modules
echo ""
echo "Installing kernel modules..."
sudo tar -xjmpf kernel_supplements.tar.bz2 -C /
sudo depmod -a
echo "✓ Modules installed"

# Install ISP tuning file
if [ -f "camera_overrides_jetson-onano.isp" ]; then
    echo ""
    echo "Installing ISP tuning file..."
    sudo mkdir -p /var/nvidia/nvcam/settings/
    sudo cp camera_overrides_jetson-onano.isp /var/nvidia/nvcam/settings/camera_overrides.isp
    sudo chmod 664 /var/nvidia/nvcam/settings/camera_overrides.isp
    sudo chown root:root /var/nvidia/nvcam/settings/camera_overrides.isp
    echo "✓ ISP tuning file installed"
fi

# Configure device tree overlay
echo ""
echo "Configuring device tree overlay..."
sudo cp /boot/extlinux/extlinux.conf /boot/extlinux/extlinux.conf.backup_$(date +%Y%m%d_%H%M%S)

# Check if overlay is already configured
if grep -q "tegra234-p3767-0000-p3768-0000-a0-4lane-ar0234.dtbo" /boot/extlinux/extlinux.conf; then
    echo "⚠ Overlay already configured in extlinux.conf"
else
    # Add overlay after FDT line
    sudo sed -i '/FDT \/boot\/dtb\/kernel_tegra234-p3767-0000-p3768-0000-a0.dtb/a\      OVERLAYS /boot/tegra234-p3767-0000-p3768-0000-a0-4lane-ar0234.dtbo' /boot/extlinux/extlinux.conf
    echo "✓ Overlay configured"
fi

echo ""
echo "=============================================="
echo "  Installation Complete!"
echo "=============================================="
echo ""
echo "Next steps:"
echo "  1. Reboot the Jetson: sudo reboot"
echo ""
echo "  2. After reboot, verify camera:"
echo "     ls /dev/video*"
echo "     lsmod | grep ar0234"
echo "     v4l2-ctl --list-devices"
echo ""
echo "  3. Test with GStreamer:"
echo "     gst-launch-1.0 v4l2src device=/dev/video0 ! \\"
echo "         'video/x-raw,width=1280,height=720,framerate=20/1' ! \\"
echo "         xvimagesink sync=false"
echo ""
echo "=============================================="
INSTALL_EOF

chmod +x "$OUTPUT_DIR/install_on_jetson.sh"

# Summary
echo ""
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}Build Complete!${NC}"
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo "Output directory: $OUTPUT_DIR"
echo ""
echo "Files ready to copy to Jetson:"
echo "  • $DTBO_FILE"
echo "  • kernel_supplements.tar.bz2"
if [ -f "$OUTPUT_DIR/camera_overrides_jetson-onano.isp" ]; then
    echo "  • camera_overrides_jetson-onano.isp"
fi
echo "  • install_on_jetson.sh"
echo ""
echo "Next steps:"
echo "  1. Copy files to Jetson:"
echo "     cd $OUTPUT_DIR"
echo "     scp * aaron@<jetson-ip>:~/"
echo ""
echo "  2. On Jetson, run:"
echo "     cd ~"
echo "     bash install_on_jetson.sh"
echo ""
echo "  3. Reboot Jetson:"
echo "     sudo reboot"
echo ""
echo "  4. Verify camera works:"
echo "     ls /dev/video0 && lsmod | grep ar0234 && echo '✅ Camera ready!'"
echo ""

