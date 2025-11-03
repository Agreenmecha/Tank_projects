# e-CAM25_CUONX Build Guide for JetPack 6.2.1 (L4T 36.4.7)

**Platform:** Build on Linux PC (Ubuntu 20.04/22.04)  
**Target:** Jetson Orin Nano with JetPack 6.2.1 (L4T 36.4.7)  
**Time Required:** 2-3 hours (first time)  
**Result:** Camera drivers with proper device tree integration

---

## Prerequisites

### On Your Linux PC:
- Ubuntu 22.04 (64-bit) or Ubuntu 20.04 (64-bit)
- At least 50GB free disk space
- Good internet connection
- `lbzip2` package installed: `sudo apt install lbzip2`

### Files You Already Have:
- e-CAM25_CUONX release package: `e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_29-MAR-2025_R04/`

---

## Step 1: Setup Environment on Linux PC

Create workspace and set environment variables:

```bash
# Create workspace directory
mkdir -p ~/jetson_camera_build
cd ~/jetson_camera_build

# Set environment variables (copy-paste all of these)
export TOP_DIR=$PWD
export RELEASE_PACK_DIR=$PWD/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_29-MAR-2025_R04
export NVIDIA_SRC=$PWD/Linux_for_Tegra/source
export LDK_ROOTFS_DIR=$PWD/Linux_for_Tegra/rootfs
export MAKEFILE_DIR=$PWD/sensor_driver
export CROSS_COMPILE=$PWD/aarch64--glibc--stable-2022.08-1/bin/aarch64-buildroot-linux-gnu-
export KERNEL_HEADERS=$NVIDIA_SRC/kernel/kernel-jammy-src
export NVIDIA_HEADERS=$NVIDIA_SRC/nvidia-oot
export NVIDIA_CONFTEST=$NVIDIA_HEADERS/conftest
```

**IMPORTANT:** Keep this terminal open! All commands must run in this same terminal.

---

## Step 2: Download Required Files

### 2.1 Download Bootlin Toolchain

```bash
cd $TOP_DIR
wget https://toolchains.bootlin.com/downloads/releases/toolchains/aarch64/tarballs/aarch64--glibc--stable-2022.08-1.tar.bz2
tar -xf aarch64--glibc--stable-2022.08-1.tar.bz2
```

### 2.2 Download NVIDIA JetPack 6.2 Sources (L4T 36.4.3)

**Note:** JetPack 6.2.1 (36.4.7) uses the same sources as JetPack 6.2 (36.4.3).

```bash
cd $TOP_DIR

# Driver Package (BSP)
wget https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v4.3/release/jetson_linux_r36.4.3_aarch64.tbz2

# Sample Root Filesystem
wget https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v4.3/release/tegra_linux_sample-root-filesystem_r36.4.3_aarch64.tbz2

# Driver Package Sources
wget https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v4.3/sources/public_sources.tbz2
```

**If wget fails (NVIDIA login required):**
1. Go to https://developer.nvidia.com/embedded/jetpack
2. Log in with NVIDIA Developer account
3. Navigate to JetPack 6.2 → L4T 36.4.3
4. Manually download the 3 files above
5. Copy them to `~/jetson_camera_build/`

### 2.3 Copy e-CAM25 Release Package

From the Jetson, copy the release package to your Linux PC:

```bash
# On Jetson, compress and send to Linux PC
cd /home/aaron/Tank_projects/ecam-25docs/
tar -czf ecam25_release.tar.gz e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_29-MAR-2025_R04/
scp ecam25_release.tar.gz <your_linux_pc_user>@<linux_pc_ip>:~/jetson_camera_build/

# On Linux PC, extract
cd ~/jetson_camera_build
tar -xzf ecam25_release.tar.gz
```

---

## Step 3: Extract and Prepare L4T

```bash
cd $TOP_DIR

# Extract driver package (creates Linux_for_Tegra directory)
sudo tar -xpf jetson_linux_r36.4.3_aarch64.tbz2

# Extract rootfs
cd Linux_for_Tegra/rootfs
sudo tar -xpf ../../tegra_linux_sample-root-filesystem_r36.4.3_aarch64.tbz2

# Apply NVIDIA binaries
cd ..
sudo ./apply_binaries.sh

# Extract kernel sources
cd $TOP_DIR
tar -xf public_sources.tbz2
cd Linux_for_Tegra/source
tar -xf kernel_src.tbz2
tar -xf kernel_oot_modules_src.tbz2
```

---

## Step 4: Apply e-CAM25 Kernel Patches

### 4.1 Apply Module Patch

```bash
cd $KERNEL_HEADERS
mkdir -p $MAKEFILE_DIR
patch -d $MAKEFILE_DIR -p1 -i $RELEASE_PACK_DIR/e-CAM25_CUONX_L4T36.4.0_JP6.1.0_JETSON-ONX-ONANO_R04/ONX_ONANO/Kernel/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_module.patch
```

### 4.2 Remove vcc-supply from Device Tree Patch

```bash
sed -i 's/vcc-supply/\/\/vcc-supply/g' $RELEASE_PACK_DIR/e-CAM25_CUONX_L4T36.4.0_JP6.1.0_JETSON-ONX-ONANO_R04/ONX_ONANO/Kernel/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_dtb.patch
```

### 4.3 Apply Device Tree Patch

```bash
patch -p1 -i $RELEASE_PACK_DIR/e-CAM25_CUONX_L4T36.4.0_JP6.1.0_JETSON-ONX-ONANO_R04/ONX_ONANO/Kernel/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_dtb.patch
```

**If you see "Reversed patch detected" message:**
- Type `n` (no) and press Enter
- This is normal and expected

### 4.4 Remove g_parm and s_parm from nvidia-oot Patch

```bash
sed '/@@ -2281,6 +2285,28 @@ static long tegra_channel_default_ioctl/,/tegra_channel_close/d' \
    $RELEASE_PACK_DIR/e-CAM25_CUONX_L4T36.4.0_JP6.1.0_JETSON-ONX-ONANO_R04/ONX_ONANO/Kernel/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_oot.patch \
    > /tmp/tempfile && \
    mv /tmp/tempfile $RELEASE_PACK_DIR/e-CAM25_CUONX_L4T36.4.0_JP6.1.0_JETSON-ONX-ONANO_R04/ONX_ONANO/Kernel/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_oot.patch
```

### 4.5 Apply nvidia-oot Patch

```bash
patch -p1 -i $RELEASE_PACK_DIR/e-CAM25_CUONX_L4T36.4.0_JP6.1.0_JETSON-ONX-ONANO_R04/ONX_ONANO/Kernel/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_oot.patch
```

### 4.6 Fix vi5_fops.c for Proper Camera Streaming

```bash
sed '238,238d' $NVIDIA_SRC/nvidia-oot/drivers/media/platform/tegra/camera/vi/vi5_fops.c > /tmp/tempfile && \
    mv /tmp/tempfile $NVIDIA_SRC/nvidia-oot/drivers/media/platform/tegra/camera/vi/vi5_fops.c
sed '228i#endif' $NVIDIA_SRC/nvidia-oot/drivers/media/platform/tegra/camera/vi/vi5_fops.c > /tmp/tempfile && \
    mv /tmp/tempfile $NVIDIA_SRC/nvidia-oot/drivers/media/platform/tegra/camera/vi/vi5_fops.c
```

---

## Step 5: Build Kernel and Modules

This takes 30-60 minutes depending on your PC.

```bash
cd $NVIDIA_SRC

# Build kernel image
make -C kernel

# Build modules (this takes a while)
make modules

# Install modules to rootfs
sudo -E make modules_install

# Build device tree blobs
make dtbs
```

**Watch for errors!** If you see compilation errors, check:
- All environment variables are set (run `echo $NVIDIA_SRC` to verify)
- All patches applied successfully
- Toolchain path is correct

---

## Step 6: Copy Built Binaries to Jetson

### 6.1 Copy DTBO

**For 4-lane configuration (recommended):**
```bash
cd $NVIDIA_SRC/kernel-devicetree/generic-dts/dtbs/
scp tegra234-p3767-0000-p3768-0000-a0-4lane-ar0234.dtbo aaron@<jetson-ip>:~/
```

**For 2-lane configuration:**
```bash
scp tegra234-p3767-0000-p3768-0000-a0-2lane-ar0234.dtbo aaron@<jetson-ip>:~/
```

### 6.2 Compress and Copy Kernel Modules

```bash
cd $LDK_ROOTFS_DIR/lib/
sudo tar -cjmpf kernel_supplements.tar.bz2 modules/
scp kernel_supplements.tar.bz2 aaron@<jetson-ip>:~/
```

---

## Step 7: Install Binaries on Jetson

**On the Jetson, run these commands:**

```bash
cd ~

# Install DTBO
sudo cp tegra234-p3767-0000-p3768-0000-a0-4lane-ar0234.dtbo /boot/

# Install kernel modules
sudo tar -xjmpf kernel_supplements.tar.bz2 -C /usr/lib/

# Update module dependencies
sudo depmod -a
```

---

## Step 8: Configure Device Tree Overlay on Jetson

**On the Jetson:**

### 8.1 Backup extlinux.conf

```bash
sudo cp /boot/extlinux/extlinux.conf /boot/extlinux/extlinux.conf.backup_$(date +%Y%m%d_%H%M%S)
```

### 8.2 Edit extlinux.conf

```bash
sudo nano /boot/extlinux/extlinux.conf
```

Find the line that says:
```
      FDT /boot/dtb/kernel_tegra234-p3767-0000-p3768-0000-a0.dtb
```

**Add this line AFTER it:**
```
      OVERLAYS /boot/tegra234-p3767-0000-p3768-0000-a0-4lane-ar0234.dtbo
```

**Final result should look like:**
```
LABEL primary
      MENU LABEL primary kernel
      LINUX /boot/Image
      FDT /boot/dtb/kernel_tegra234-p3767-0000-p3768-0000-a0.dtb
      OVERLAYS /boot/tegra234-p3767-0000-p3768-0000-a0-4lane-ar0234.dtbo
      INITRD /boot/initrd
      APPEND ${cbootargs} root=...
```

Save (Ctrl+O, Enter) and exit (Ctrl+X).

---

## Step 9: Reboot and Test

```bash
sudo reboot
```

**After reboot, test the camera:**

```bash
# Check if camera device exists
ls /dev/video*
# Should show: /dev/video0

# Check if modules loaded
lsmod | grep ar0234

# Verify device tree nodes
ls /sys/firmware/devicetree/base/i2c*/ar0234* 2>/dev/null

# Test with v4l2
v4l2-ctl --list-devices
v4l2-ctl --all --device /dev/video0
```

---

## Troubleshooting

### Problem: Compilation Errors

**Solution:**
- Verify environment variables: `echo $NVIDIA_SRC`
- Re-run the environment setup commands (Step 1)
- Check if all patches applied without errors

### Problem: No /dev/video* After Reboot

**Solution:**
```bash
# Check if ar0234 module loaded
lsmod | grep ar0234

# If not, manually load it
sudo modprobe ar0234

# Check dmesg for errors
sudo dmesg | grep -i "ar0234\|camera\|vi5"

# Verify device tree was applied
ls /sys/firmware/devicetree/base/i2c*/ar0234*
```

### Problem: "module verification failed" Warning

**This is normal!** The warning about signature is harmless and doesn't affect functionality.

---

## Success Criteria

✅ `/dev/video0` exists  
✅ `lsmod | grep ar0234` shows the driver  
✅ `v4l2-ctl --list-devices` shows "vi-output" device  
✅ Device tree nodes exist in `/sys/firmware/devicetree/base/`

---

## Next Steps After Success

1. Test camera streaming with GStreamer
2. Configure for ROS2 (gscam or custom node)
3. Integrate with tank_sensors package

---

## Quick Reference Commands

**Check if everything worked:**
```bash
# One-liner to verify camera is ready
ls /dev/video0 && lsmod | grep ar0234 && echo "✅ Camera ready!"
```

**Start over if needed:**
```bash
# On Jetson, remove installed files
sudo rm /boot/tegra234-p3767-0000-p3768-0000-a0-4lane-ar0234.dtbo
sudo rm /lib/modules/5.15.148-tegra/kernel/drivers/media/i2c/ar0234.ko
sudo rm /lib/modules/5.15.148-tegra/kernel/drivers/media/platform/tegra/camera/tegra-camera.ko
sudo rm /lib/modules/5.15.148-tegra/kernel/drivers/video/tegra/host/capture-ivc.ko
sudo depmod -a

# Restore backup extlinux.conf
sudo cp /boot/extlinux/extlinux.conf.backup_XXXXXXXX /boot/extlinux/extlinux.conf
```

---

**Last Updated:** November 2, 2025  
**Tested On:** JetPack 6.2.1 (L4T 36.4.7), Jetson Orin Nano  
**Source:** e-con Systems upgrade guide + Tank project integration

