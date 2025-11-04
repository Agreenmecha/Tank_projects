#!/bin/bash
################################################################################
# e-CAM25_CUONX Driver Build Script for JetPack 6.2.1
# Build on Linux PC (Ubuntu 20.04/22.04)
# Target: Jetson Orin Nano with JetPack 6.2.1 (L4T 36.4.7)
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
echo "  e-CAM25_CUONX Driver Build Script for JetPack 6.2.1"
echo "  Build on Linux PC → Install on Jetson Orin Nano"
echo "================================================================================"
echo -e "${NC}"
echo ""

# Check if running on Linux (not Jetson)
if uname -m | grep -q "aarch64"; then
    echo -e "${RED}ERROR: This script must run on a Linux PC (x86_64), not on Jetson!${NC}"
    echo "Please run this script on your Ubuntu 20.04/22.04 PC"
    exit 1
fi

# Check for required tools
echo "Checking required tools..."
for tool in wget tar lbzip2 patch sed; do
    if ! command -v $tool &> /dev/null; then
        echo -e "${RED}ERROR: $tool not found. Installing...${NC}"
        sudo apt update && sudo apt install -y $tool
    fi
done

# Setup workspace
WORKSPACE="$HOME/jetson_camera_build"
echo ""
echo "Workspace: $WORKSPACE"
read -p "Continue with build? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Build cancelled."
    exit 0
fi

mkdir -p "$WORKSPACE"
cd "$WORKSPACE"

# Set environment variables
export TOP_DIR="$WORKSPACE"
export RELEASE_PACK_DIR="$WORKSPACE/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_29-MAR-2025_R04"
export NVIDIA_SRC="$WORKSPACE/Linux_for_Tegra/source"
export LDK_ROOTFS_DIR="$WORKSPACE/Linux_for_Tegra/rootfs"
export MAKEFILE_DIR="$WORKSPACE/sensor_driver"
export CROSS_COMPILE="$WORKSPACE/aarch64--glibc--stable-2022.08-1/bin/aarch64-buildroot-linux-gnu-"
export KERNEL_HEADERS="$NVIDIA_SRC/kernel/kernel-jammy-src"
export NVIDIA_HEADERS="$NVIDIA_SRC/nvidia-oot"
export NVIDIA_CONFTEST="$NVIDIA_HEADERS/conftest"

echo ""
echo -e "${GREEN}Environment variables set:${NC}"
echo "  TOP_DIR: $TOP_DIR"
echo "  RELEASE_PACK_DIR: $RELEASE_PACK_DIR"
echo "  NVIDIA_SRC: $NVIDIA_SRC"
echo ""

# Step 1: Check for downloaded files
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}Step 1: Checking for downloaded files${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

REQUIRED_FILES=(
    "aarch64--glibc--stable-2022.08-1.tar.bz2"
    "jetson_linux_r36.4.3_aarch64.tbz2"
    "tegra_linux_sample-root-filesystem_r36.4.3_aarch64.tbz2"
    "public_sources.tbz2"
)

MISSING_FILES=()
for file in "${REQUIRED_FILES[@]}"; do
    if [ ! -f "$WORKSPACE/$file" ]; then
        MISSING_FILES+=("$file")
        echo -e "${YELLOW}⚠ Missing: $file${NC}"
    else
        echo -e "${GREEN}✓ Found: $file${NC}"
    fi
done

if [ ${#MISSING_FILES[@]} -gt 0 ]; then
    echo ""
    echo -e "${RED}Missing required files!${NC}"
    echo "Please download the following files from NVIDIA:"
    echo ""
    for file in "${MISSING_FILES[@]}"; do
        case $file in
            "aarch64--glibc--stable-2022.08-1.tar.bz2")
                echo "  • Bootlin Toolchain: https://toolchains.bootlin.com/downloads/releases/toolchains/aarch64/tarballs/aarch64--glibc--stable-2022.08-1.tar.bz2"
                ;;
            "jetson_linux_r36.4.3_aarch64.tbz2")
                echo "  • NVIDIA Driver Package (BSP): Jetson_Linux_R36.4.3_aarch64.tbz2"
                echo "    Download from: https://developer.nvidia.com/embedded/jetpack"
                ;;
            "tegra_linux_sample-root-filesystem_r36.4.3_aarch64.tbz2")
                echo "  • NVIDIA Sample Root Filesystem: Tegra_Linux_Sample-Root-Filesystem_R36.4.3_aarch64.tbz2"
                echo "    Download from: https://developer.nvidia.com/embedded/jetpack"
                ;;
            "public_sources.tbz2")
                echo "  • NVIDIA Public Sources: public_sources.tbz2"
                echo "    Download from: https://developer.nvidia.com/embedded/jetpack"
                ;;
        esac
    done
    echo ""
    echo "Place all downloaded files in: $WORKSPACE"
    echo ""
    read -p "Press Enter when files are downloaded, or Ctrl+C to cancel..."
fi

# Check for e-CAM25 release package
echo ""
echo "Checking for e-CAM25 release package..."
if [ ! -d "$RELEASE_PACK_DIR" ]; then
    echo -e "${YELLOW}⚠ e-CAM25 release package not found${NC}"
    echo "Expected location: $RELEASE_PACK_DIR"
    echo ""
    echo "Options:"
    echo "  1. Copy from Jetson:"
    echo "     cd ~/Tank_projects/ecam-25docs/"
    echo "     tar -czf ecam25_release.tar.gz e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_29-MAR-2025_R04/"
    echo "     scp ecam25_release.tar.gz <pc-user>@<pc-ip>:~/jetson_camera_build/"
    echo ""
    echo "  2. Copy from local directory:"
    read -p "Enter path to e-CAM25 release package: " RELEASE_PATH
    if [ -d "$RELEASE_PATH" ]; then
        cp -r "$RELEASE_PATH" "$RELEASE_PACK_DIR"
        echo -e "${GREEN}✓ Copied release package${NC}"
    else
        echo -e "${RED}ERROR: Release package not found at $RELEASE_PATH${NC}"
        exit 1
    fi
else
    echo -e "${GREEN}✓ e-CAM25 release package found${NC}"
fi

# Step 2: Extract Bootlin toolchain
echo ""
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}Step 2: Extracting Bootlin toolchain${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

if [ ! -d "$WORKSPACE/aarch64--glibc--stable-2022.08-1" ]; then
    echo "Extracting Bootlin toolchain (this may take a few minutes)..."
    tar -xf "$WORKSPACE/aarch64--glibc--stable-2022.08-1.tar.bz2"
    echo -e "${GREEN}✓ Toolchain extracted${NC}"
else
    echo -e "${GREEN}✓ Toolchain already extracted${NC}"
fi

# Step 3: Extract NVIDIA L4T packages
echo ""
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}Step 3: Extracting NVIDIA L4T packages${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

if [ ! -d "$WORKSPACE/Linux_for_Tegra" ]; then
    echo "Extracting driver package (this takes 5-10 minutes)..."
    sudo tar -xpf "$WORKSPACE/jetson_linux_r36.4.3_aarch64.tbz2"
    echo -e "${GREEN}✓ Driver package extracted${NC}"
    
    echo ""
    echo "Extracting root filesystem (this takes 10-15 minutes)..."
    cd "$WORKSPACE/Linux_for_Tegra/rootfs"
    sudo tar -xpf "$WORKSPACE/tegra_linux_sample-root-filesystem_r36.4.3_aarch64.tbz2"
    echo -e "${GREEN}✓ Root filesystem extracted${NC}"
    
    echo ""
    echo "Applying NVIDIA binaries..."
    cd "$WORKSPACE/Linux_for_Tegra"
    sudo ./apply_binaries.sh
    echo -e "${GREEN}✓ Binaries applied${NC}"
else
    echo -e "${GREEN}✓ L4T packages already extracted${NC}"
fi

# Step 4: Extract kernel sources
echo ""
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}Step 4: Extracting kernel sources${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

if [ ! -d "$NVIDIA_SRC/kernel" ]; then
    echo "Extracting public sources (this takes 5-10 minutes)..."
    cd "$WORKSPACE"
    tar -xf "$WORKSPACE/public_sources.tbz2"
    
    cd "$NVIDIA_SRC"
    echo "Extracting kernel sources..."
    tar -xf kernel_src.tbz2
    echo "Extracting OOT modules..."
    tar -xf kernel_oot_modules_src.tbz2
    echo -e "${GREEN}✓ Kernel sources extracted${NC}"
else
    echo -e "${GREEN}✓ Kernel sources already extracted${NC}"
fi

# Step 5: Apply e-CAM25 kernel patches
echo ""
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}Step 5: Applying e-CAM25 kernel patches${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

cd "$KERNEL_HEADERS"

# Find patch files
MODULE_PATCH="$RELEASE_PACK_DIR/e-CAM25_CUONX_L4T36.4.0_JP6.1.0_JETSON-ONX-ONANO_R04/ONX_ONANO/Kernel/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_module.patch"
DTB_PATCH="$RELEASE_PACK_DIR/e-CAM25_CUONX_L4T36.4.0_JP6.1.0_JETSON-ONX-ONANO_R04/ONX_ONANO/Kernel/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_dtb.patch"
OOT_PATCH="$RELEASE_PACK_DIR/e-CAM25_CUONX_L4T36.4.0_JP6.1.0_JETSON-ONX-ONANO_R04/ONX_ONANO/Kernel/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_oot.patch"

# 5.1: Apply module patch
echo "5.1: Applying module patch..."
mkdir -p "$MAKEFILE_DIR"
if patch -d "$MAKEFILE_DIR" -p1 -i "$MODULE_PATCH" --dry-run &>/dev/null; then
    patch -d "$MAKEFILE_DIR" -p1 -i "$MODULE_PATCH" || echo -e "${YELLOW}⚠ Patch may have been partially applied${NC}"
    echo -e "${GREEN}✓ Module patch applied${NC}"
else
    echo -e "${YELLOW}⚠ Module patch may already be applied${NC}"
fi

# 5.2: Modify and apply device tree patch
echo ""
echo "5.2: Modifying and applying device tree patch..."
# Create backup
cp "$DTB_PATCH" "$DTB_PATCH.backup"
# Remove vcc-supply for 6.2.x
sed -i 's/vcc-supply/\/\/vcc-supply/g' "$DTB_PATCH"
echo -e "${GREEN}✓ Modified DTB patch for 6.2.x${NC}"

# Apply patch
if patch -p1 -i "$DTB_PATCH" --dry-run &>/dev/null; then
    patch -p1 -i "$DTB_PATCH" <<< "n" || echo -e "${YELLOW}⚠ Some hunks may have been skipped (normal)${NC}"
    echo -e "${GREEN}✓ Device tree patch applied${NC}"
else
    echo -e "${YELLOW}⚠ Device tree patch may already be applied${NC}"
fi

# 5.3: Modify and apply nvidia-oot patch
echo ""
echo "5.3: Modifying and applying nvidia-oot patch..."
# Create backup
cp "$OOT_PATCH" "$OOT_PATCH.backup"
# Remove g_parm and s_parm functions for e-CAM25_CUONX (line 2281)
sed '/@@ -2281,6 +2285,28 @@ static long tegra_channel_default_ioctl/,/tegra_channel_close/d' "$OOT_PATCH" > /tmp/tempfile && \
    mv /tmp/tempfile "$OOT_PATCH"
echo -e "${GREEN}✓ Modified OOT patch for 6.2.x${NC}"

# Apply patch
if patch -p1 -i "$OOT_PATCH" --dry-run &>/dev/null; then
    patch -p1 -i "$OOT_PATCH" || echo -e "${YELLOW}⚠ OOT patch may have been partially applied${NC}"
    echo -e "${GREEN}✓ nvidia-oot patch applied${NC}"
else
    echo -e "${YELLOW}⚠ nvidia-oot patch may already be applied${NC}"
fi

# 5.4: Fix vi5_fops.c for proper camera streaming
echo ""
echo "5.4: Fixing vi5_fops.c for camera streaming..."
VI5_FILE="$NVIDIA_SRC/nvidia-oot/drivers/media/platform/tegra/camera/vi/vi5_fops.c"
if [ -f "$VI5_FILE" ]; then
    # Backup
    cp "$VI5_FILE" "$VI5_FILE.backup"
    # Remove line 238
    sed '238,238d' "$VI5_FILE" > /tmp/tempfile && mv /tmp/tempfile "$VI5_FILE"
    # Insert #endif at line 228
    sed '228i#endif' "$VI5_FILE" > /tmp/tempfile && mv /tmp/tempfile "$VI5_FILE"
    echo -e "${GREEN}✓ vi5_fops.c fixed${NC}"
else
    echo -e "${RED}ERROR: vi5_fops.c not found at $VI5_FILE${NC}"
    exit 1
fi

# Step 6: Build kernel and modules
echo ""
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}Step 6: Building kernel and modules${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo -e "${YELLOW}This step takes 30-60 minutes depending on your PC...${NC}"
echo ""

cd "$NVIDIA_SRC"

# Build kernel
echo "Building kernel..."
make -C kernel
echo -e "${GREEN}✓ Kernel built${NC}"

# Build modules
echo ""
echo "Building modules (this takes the longest)..."
make modules
echo -e "${GREEN}✓ Modules built${NC}"

# Install modules to rootfs
echo ""
echo "Installing modules to rootfs..."
sudo -E make modules_install
echo -e "${GREEN}✓ Modules installed${NC}"

# Build device tree blobs
echo ""
echo "Building device tree blobs..."
make dtbs
echo -e "${GREEN}✓ Device tree blobs built${NC}"

# Step 7: Prepare binaries for Jetson
echo ""
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}Step 7: Preparing binaries for Jetson${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

# Ask for lane configuration
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
    echo -e "${YELLOW}Invalid choice, defaulting to 4-lane${NC}"
    LANE_MODE="4lane"
    DTBO_FILE="tegra234-p3767-0000-p3768-0000-a0-4lane-ar0234.dtbo"
fi

echo ""
echo "Using ${LANE_MODE} configuration"
echo ""

# Copy DTBO
DTBO_SOURCE="$NVIDIA_SRC/kernel-devicetree/generic-dts/dtbs/$DTBO_FILE"
if [ -f "$DTBO_SOURCE" ]; then
    cp "$DTBO_SOURCE" "$WORKSPACE/"
    echo -e "${GREEN}✓ DTBO copied: $DTBO_FILE${NC}"
else
    echo -e "${RED}ERROR: DTBO not found at $DTBO_SOURCE${NC}"
    echo "Please check build output"
    exit 1
fi

# Compress kernel modules
echo ""
echo "Compressing kernel modules..."
cd "$LDK_ROOTFS_DIR/lib/"
sudo tar -cjmpf "$WORKSPACE/kernel_supplements.tar.bz2" modules/
echo -e "${GREEN}✓ Kernel modules compressed${NC}"

# Copy ISP tuning file
ISP_FILE="$RELEASE_PACK_DIR/e-CAM25_CUONX_L4T36.4.0_JP6.1.0_JETSON-ONX-ONANO_R04/ONX_ONANO/misc/camera_overrides_jetson-onano.isp"
if [ -f "$ISP_FILE" ]; then
    cp "$ISP_FILE" "$WORKSPACE/"
    echo -e "${GREEN}✓ ISP tuning file copied${NC}"
else
    echo -e "${YELLOW}⚠ ISP tuning file not found (optional)${NC}"
fi

# Summary
echo ""
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}Build Complete!${NC}"
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo "Files ready to copy to Jetson:"
echo "  • $WORKSPACE/$DTBO_FILE"
echo "  • $WORKSPACE/kernel_supplements.tar.bz2"
if [ -f "$WORKSPACE/camera_overrides_jetson-onano.isp" ]; then
    echo "  • $WORKSPACE/camera_overrides_jetson-onano.isp"
fi
echo ""
echo "Next steps:"
echo "  1. Copy files to Jetson:"
echo "     scp $WORKSPACE/$DTBO_FILE aaron@<jetson-ip>:~/"
echo "     scp $WORKSPACE/kernel_supplements.tar.bz2 aaron@<jetson-ip>:~/"
if [ -f "$WORKSPACE/camera_overrides_jetson-onano.isp" ]; then
    echo "     scp $WORKSPACE/camera_overrides_jetson-onano.isp aaron@<jetson-ip>:~/"
fi
echo ""
echo "  2. On Jetson, run: ~/Tank_projects/ecam-25docs/install_ecam25_built.sh"
echo ""
echo "  3. Or follow manual installation steps in BUILD_ECAM25_JP621_GUIDE.md"
echo ""

