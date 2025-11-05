# Building e-CAM25_CUONX Drivers on Native Linux PC

**Repository-Based Build Guide for Tank Project**

**Important:** Camera driver compilation MUST be done on a native Linux PC (Ubuntu 20.04/22.04), not in WSL.

**Time Required:** 1.5-2.5 hours (first time)  
**Target:** Jetson Orin Nano with JetPack 6.2.1 (L4T 36.4.7)  
**Camera:** e-CAM25_CUONX (AR0234 global shutter)

---

## Prerequisites on Native Linux PC

### System Requirements:
- **OS:** Ubuntu 20.04 LTS or Ubuntu 22.04 LTS (64-bit)
- **Disk Space:** At least 50GB free
- **RAM:** 8GB minimum, 16GB recommended
- **CPU:** Multi-core processor (build uses parallel compilation)
- **Git:** For cloning the Tank_projects repository

### Install Required Tools:

```bash
sudo apt update
sudo apt install -y build-essential bc bison flex libssl-dev \
    libncurses-dev git wget curl tar bzip2 lbzip2 \
    python3 python3-pip device-tree-compiler
```

---

## Step 1: Clone Repository

**On your Linux PC:**

```bash
cd ~
git clone https://github.com/YOUR_USERNAME/Tank_projects.git
cd Tank_projects/camera_build_workspace
```

---

## Step 2: Download Required Files

The repository doesn't include large binary files (they're in .gitignore). You need to download them separately.

### 2.1: Download Bootlin Toolchain

```bash
cd ~/Tank_projects/camera_build_workspace
wget https://toolchains.bootlin.com/downloads/releases/toolchains/aarch64/tarballs/aarch64--glibc--stable-2022.08-1.tar.bz2
```

### 2.2: Download NVIDIA Files

**Important:** These files are also needed but may already be in `../nvidiaR36.4.3/` in the repo. If not, download them:

```bash
cd ~/Tank_projects/nvidiaR36.4.3
```

Download these files from NVIDIA (requires free NVIDIA Developer account):
- Go to: https://developer.nvidia.com/embedded/jetpack
- Select: JetPack 6.2 → L4T 36.4.3
- Download:
  - **Jetson Linux R36.4.3** (Driver Package) → `Jetson_Linux_R36.4.3_aarch64.tbz2` (683MB)
  - **Sample Root Filesystem** → `Tegra_Linux_Sample-Root-Filesystem_R36.4.3_aarch64.tbz2` (1.8GB)
  - **Sources** → `public_sources.tbz2` (216MB)

### 2.3: Verify e-CAM25 Release Package

The e-CAM25 release package should be in the repository. Verify it's present:

```bash
cd ~/Tank_projects
ls -d e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_29-MAR-2025_R04
```

If missing, you need to obtain it from e-con Systems or copy it to the repo.

---

## Step 3: Run the Build Script

The automated build script handles everything.

**On your Linux PC:**

```bash
cd ~/Tank_projects/camera_build_workspace
./scripts/build_drivers.sh
```

The script will:
1. ✓ Extract Bootlin toolchain (2 minutes)
2. ✓ Extract NVIDIA L4T packages (15-20 minutes)
3. ✓ Extract kernel sources (5-10 minutes)
4. ✓ Apply e-CAM25 patches with JetPack 6.2.x modifications (1-2 minutes)
5. ✓ Build kernel and modules (30-60 minutes)
6. ✓ Build device tree overlays (2 minutes)
7. ✓ Package binaries for Jetson (5 minutes)

**Total time: 50-100 minutes**

### Build Output

When complete, you'll see:

```
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Build Complete!
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Output directory: ~/Tank_projects/camera_build_workspace/jetson_binaries

Files ready to copy to Jetson:
  • tegra234-p3767-0000-p3768-0000-a0-4lane-ar0234.dtbo
  • kernel_supplements.tar.bz2
  • camera_overrides_jetson-onano.isp
  • install_on_jetson.sh
```

---

## Step 4: Transfer Built Binaries to Jetson

**On your Linux PC:**

```bash
cd ~/Tank_projects/camera_build_workspace/jetson_binaries

# Check what was built
ls -lh

# Transfer to Jetson (replace with your Jetson's IP)
scp * aaron@<jetson-ip>:~/
```

**Alternative:** Copy to USB drive if no network connection.

---

## Step 5: Install on Jetson

**On the Jetson Orin Nano:**

```bash
cd ~

# Verify files are present
ls -lh *.dtbo *.tar.bz2 *.sh

# Run installation script
bash install_on_jetson.sh

# The script will:
# - Install device tree overlay to /boot/
# - Install kernel modules to /lib/modules/
# - Install ISP tuning file to /var/nvidia/nvcam/settings/
# - Configure extlinux.conf to load the overlay

# Reboot to apply changes
sudo reboot
```

---

## Step 6: Verify Installation

**After Jetson reboots:**

```bash
# Check if camera device exists
ls /dev/video*
# Expected: /dev/video0

# Check if driver is loaded
lsmod | grep ar0234
# Expected: ar0234 module listed

# Check device tree
ls /sys/firmware/devicetree/base/i2c*/ar0234* 2>/dev/null
# Expected: ar0234_a device node

# Test with v4l2-ctl
v4l2-ctl --list-devices
# Expected: vi-output device

# Test camera stream (basic test)
v4l2-ctl --device /dev/video0 --stream-mmap --stream-count=10
```

**Expected output:**
```
✅ /dev/video0 exists
✅ ar0234 module loaded
✅ Device tree nodes present
✅ Camera responds to v4l2 commands
```

---

## Step 7: Commit Changes (Optional)

If you made any modifications, commit them to the repository:

```bash
cd ~/Tank_projects

# Check what changed
git status

# Add documentation or script changes (not binaries!)
git add camera_build_workspace/docs/
git add camera_build_workspace/scripts/

# Commit
git commit -m "Updated camera driver build documentation"

# Push to GitHub
git push origin main
```

**Note:** Large binary files (.tbz2, .tar.bz2) are in .gitignore and won't be committed.

---

## Troubleshooting

### Build Errors

**If kernel build fails:**
```bash
# Check build log
cd ~/Tank_projects/camera_build_workspace
tail -100 build.log

# Clean and retry
cd Linux_for_Tegra/source
make clean
cd ~/Tank_projects/camera_build_workspace
./scripts/build_drivers.sh
```

**If patches fail to apply:**
```bash
# Check if patches were already applied
ls -la sensor_driver/.module_patch_applied
ls -la Linux_for_Tegra/source/kernel/kernel-jammy-src/.dtb_patch_applied

# Remove marker files and retry
rm -f sensor_driver/.module_patch_applied
rm -f Linux_for_Tegra/source/kernel/kernel-jammy-src/.dtb_patch_applied
rm -f Linux_for_Tegra/source/kernel/kernel-jammy-src/.oot_patch_applied
./scripts/build_drivers.sh
```

**If running on WSL:**
```bash
# Error: Cannot build on WSL!
# Solution: Transfer to native Linux PC

# From WSL, package everything:
cd ~/Tank_projects
tar -czf tank_projects.tar.gz \
    camera_build_workspace/ \
    nvidiaR36.4.3/ \
    e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_29-MAR-2025_R04/

# Copy to native PC and extract
```

### Installation Issues on Jetson

**No /dev/video0 after reboot:**
```bash
# Check dmesg for errors
sudo dmesg | grep -i "ar0234\|camera\|vi5"

# Look for specific errors:
# - "probe failed" → Camera not connected or I2C issue
# - "no such device" → Device tree not loaded
# - "invalid argument" → Patch mismatch

# Manually load module
sudo modprobe ar0234

# Check module info
modinfo ar0234
```

**Module signature warnings (harmless):**
```bash
# These warnings are normal and can be ignored:
# "module verification failed: signature and/or required key missing"
# This happens because modules are built outside NVIDIA's signing process
```

**Device tree not loading:**
```bash
# Verify DTBO is in /boot
ls -l /boot/*.dtbo

# Verify extlinux.conf has OVERLAYS line
grep -A2 -B2 OVERLAYS /boot/extlinux/extlinux.conf

# Should show:
# FDT /boot/dtb/kernel_tegra234-p3767-0000-p3768-0000-a0.dtb
# OVERLAYS /boot/tegra234-p3767-0000-p3768-0000-a0-4lane-ar0234.dtbo

# If missing, manually add it
sudo nano /boot/extlinux/extlinux.conf
```

**Camera not detected on I2C:**
```bash
# Check I2C bus
sudo i2cdetect -y -r 9

# AR0234 should appear at address 0x10
# If not visible:
# - Check camera cable connection
# - Verify camera power (12V for e-CAM25_CUONX)
# - Check device tree configuration
```

### ROS2 Integration Issues

**GStreamer works but ROS2 doesn't:**
```bash
# Verify ROS2 workspace
cd ~/Tank_projects/tank_ws
colcon build --packages-select gscam

# Source workspace
source install/setup.bash

# Check gscam config
ros2 param list /gscam_node

# Test with ros2 run
ros2 run gscam gscam_node
```

---

## Build Script Recovery

If something goes wrong mid-build:

```bash
cd ~/Tank_projects/camera_build_workspace

# The script uses marker files to track progress
# To restart from scratch:
rm -rf Linux_for_Tegra/
rm -rf aarch64--glibc--stable-2022.08-1/
rm -rf sensor_driver/
rm -rf jetson_binaries/

# Re-run build script
./scripts/build_drivers.sh
```

To resume from a specific point:
- Marker files prevent re-running completed steps
- Delete specific marker file to re-run that step:
  - `.module_patch_applied` - Module patch
  - `.dtb_patch_applied` - Device tree patch
  - `.oot_patch_applied` - nvidia-oot patch

---

## Quick Reference

### Build Commands
```bash
cd ~/Tank_projects/camera_build_workspace
./scripts/build_drivers.sh
```

### Transfer to Jetson
```bash
cd ~/Tank_projects/camera_build_workspace/jetson_binaries
scp * aaron@<jetson-ip>:~/
```

### Install on Jetson
```bash
bash install_on_jetson.sh
sudo reboot
```

### Verify on Jetson
```bash
ls /dev/video0 && lsmod | grep ar0234 && echo "✅ Camera ready!"
v4l2-ctl --list-devices
```

### Test with GStreamer
```bash
gst-launch-1.0 v4l2src device=/dev/video0 ! \
    'video/x-raw,width=1280,height=720,framerate=20/1' ! \
    xvimagesink sync=false
```

### Test with ROS2
```bash
cd ~/Tank_projects/tank_ws
source install/setup.bash
ros2 launch gscam gscam.launch.py
```

---

## Files Generated

### On Linux PC:
- `~/Tank_projects/camera_build_workspace/jetson_binaries/`
  - `tegra234-p3767-0000-p3768-0000-a0-4lane-ar0234.dtbo` - Device tree overlay
  - `kernel_supplements.tar.bz2` - Kernel modules (ar0234.ko, etc.)
  - `camera_overrides_jetson-onano.isp` - ISP tuning
  - `install_on_jetson.sh` - Installation script

### On Jetson (after installation):
- `/boot/tegra234-p3767-0000-p3768-0000-a0-4lane-ar0234.dtbo`
- `/lib/modules/5.15.148-tegra/kernel/drivers/media/i2c/ar0234.ko`
- `/lib/modules/5.15.148-tegra/kernel/drivers/media/platform/tegra/camera/tegra-camera.ko`
- `/var/nvidia/nvcam/settings/camera_overrides.isp`
- `/boot/extlinux/extlinux.conf` (modified)

---

## Success Criteria

✅ **Build successful if:**
- Script completes without errors
- `jetson_binaries/` directory contains all files
- No "ERROR" messages in output

✅ **Installation successful if:**
- `/dev/video0` exists after reboot
- `lsmod | grep ar0234` shows module loaded
- `v4l2-ctl --list-devices` shows vi-output device

✅ **Camera working if:**
- Can capture frames with v4l2-ctl
- Can stream with GStreamer
- ROS2 gscam node can read camera

---

## Time Estimates

| Step | Time |
|------|------|
| Clone repository | 5 minutes |
| Download files | 10-20 minutes |
| Extract packages | 15-20 minutes |
| Apply patches | 1-2 minutes |
| Build kernel & modules | 30-60 minutes |
| Package binaries | 5-10 minutes |
| Transfer to Jetson | 2-5 minutes |
| Install on Jetson | 5 minutes |
| **Total** | **1.5-2.5 hours** |

---

## Repository Structure

```
Tank_projects/
├── camera_build_workspace/      ← You are here
│   ├── docs/
│   │   ├── BUILD_GUIDE_NATIVE_PC.md    ← This file
│   │   ├── QUICK_START.txt
│   │   └── TROUBLESHOOTING.md
│   ├── scripts/
│   │   ├── build_drivers.sh            ← Main build script
│   │   └── install_on_jetson.sh
│   ├── .gitignore                      ← Excludes large binaries
│   └── README.md
├── nvidiaR36.4.3/               ← NVIDIA downloads (not in git)
│   ├── Jetson_Linux_R36.4.3_aarch64.tbz2
│   ├── Tegra_Linux_Sample-Root-Filesystem_R36.4.3_aarch64.tbz2
│   └── public_sources.tbz2
├── e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_29-MAR-2025_R04/  ← Camera package
├── ecam-25docs/                 ← Reference documentation
├── drivers/                     ← Pre-built drivers (fallback)
└── PROJECT_STATUS.md            ← Overall project status
```

---

## Support

**If you encounter issues:**

1. Check build log: `~/Tank_projects/camera_build_workspace/build.log`
2. Read `docs/TROUBLESHOOTING.md`
3. Verify all prerequisite packages are installed
4. Ensure you're on native Linux (not WSL/VM)
5. Check disk space: `df -h ~`

**Documentation references:**
- This guide: `docs/BUILD_GUIDE_NATIVE_PC.md`
- Quick start: `docs/QUICK_START.txt`
- e-con guide: `../ecam-25docs/econ25_upgradeto6.2.txt`
- NVIDIA guide: `../ecam-25docs/BUILD_ECAM25_JP621_GUIDE.md`

---

**Good luck with the build! This should work perfectly on native Linux.** 🚀

