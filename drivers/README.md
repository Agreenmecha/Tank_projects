# e-CAM25_CUONX Camera Drivers - Deployment Package

**Built:** November 2, 2025  
**Platform:** JetPack 6.2.1 (L4T 36.4.7)  
**Source:** L4T 36.4.3 (compatible)  
**Target:** Jetson Orin Nano Development Kit (p3768-0000/p3767-0000)

## Contents

This directory contains the e-CAM25_CUONX camera device tree overlays:

- `install_camera_drivers.sh` - Installation script for Jetson
- `tegra234-p3767-0000-p3768-0000-a0-2lane-ar0234.dtbo` - 2-lane device tree overlay
- `tegra234-p3767-0000-p3768-0000-a0-4lane-ar0234.dtbo` - 4-lane device tree overlay (recommended)

## Lane Configuration

**4-Lane (Recommended):**
- Up to 70fps @ 1920x1080
- Better performance and bandwidth
- Uses CSI connector fully

**2-Lane:**
- Up to 30fps @ 1920x1200
- Lower bandwidth requirements
- Alternative if 4-lane has issues

## Installation on Jetson Orin Nano

### Method 1: Quick Install (Recommended)

1. Copy this entire directory to your Jetson:
   ```bash
   # From your PC (in WSL or another machine):
   scp -r ecam25_drivers_deploy aaron@<jetson-ip>:~/
   ```

2. On the Jetson, run the installation script:
   ```bash
   cd ~/ecam25_drivers_deploy
   sudo ./install_camera_drivers.sh 4lane    # For 4-lane (recommended)
   # OR
   sudo ./install_camera_drivers.sh 2lane    # For 2-lane
   ```

3. Reboot:
   ```bash
   sudo reboot
   ```

### Method 2: Manual Installation

If you prefer to install manually or need to troubleshoot:

```bash
# 1. Backup extlinux.conf
sudo cp /boot/extlinux/extlinux.conf /boot/extlinux/extlinux.conf.backup

# 2. Copy DTBO to /boot (choose 2lane or 4lane)
sudo cp tegra234-p3767-0000-p3768-0000-a0-4lane-ar0234.dtbo /boot/

# 3. Edit /boot/extlinux/extlinux.conf
sudo nano /boot/extlinux/extlinux.conf

# Add this line after the existing FDT line:
#      FDT /boot/tegra234-p3767-0000-p3768-0000-a0-4lane-ar0234.dtbo

# 4. Reboot
sudo reboot
```

## Verification After Reboot

Check if the camera is detected:

```bash
# List video devices
ls /dev/video*

# Check V4L2 devices
v4l2-ctl --list-devices

# Test camera with GStreamer (if installed)
gst-launch-1.0 v4l2src device=/dev/video0 ! xvimagesink

# Or use nvgstcapture (NVIDIA tool)
nvgstcapture-1.0
```

Expected output should show ar0234 or e-CAM25 device.

## Troubleshooting

### Camera not detected

1. **Check kernel log for errors:**
   ```bash
   dmesg | grep -i ar0234
   dmesg | grep -i camera
   ```

2. **Verify DTBO is loaded:**
   ```bash
   cat /boot/extlinux/extlinux.conf | grep ar0234
   ```

3. **Check I2C devices:**
   ```bash
   sudo i2cdetect -y -r 7    # CSI camera I2C bus
   ```

### Switching between 2-lane and 4-lane

```bash
# Run the install script with different config
sudo ./install_camera_drivers.sh 2lane  # or 4lane
sudo reboot
```

### Restore backup

```bash
# List backups
ls /boot/extlinux/extlinux.conf.backup_*

# Restore a backup
sudo cp /boot/extlinux/extlinux.conf.backup_YYYYMMDD_HHMMSS /boot/extlinux/extlinux.conf
sudo reboot
```

## Build Information

These drivers were built in WSL from:
- **Kernel:** L4T 36.4.3 sources
- **Patches:** e-con Systems e-CAM25_CUONX release (March 29, 2025)
- **Toolchain:** Bootlin aarch64 glibc stable 2022.08-1
- **Build Location:** `/home/agreen/.cursor/worktrees/Tank_projects__WSL__Ubuntu_/FXyVh/jetson_camera_build`

### What Was Built
- ✅ Kernel Image (5.15.148-tegra)
- ✅ Device Tree Overlays (DTBO) for 2-lane and 4-lane
- ✅ Camera patches applied (dtb, oot, vi5_fops)

### What's Missing (Advanced Users Only)
The e-CAM25_CUONX sensor driver module (ar0234.ko) was not included because:
- The nvidia-oot tree already includes nv_ar0234.c which should work
- Building the e-con module requires additional symbol dependencies
- The DTBO-only approach should work for basic camera detection

If you need the custom sensor module, you'll need to build the full kernel on the Jetson or contact e-con Systems for JetPack 6.2.1 specific binaries.

## Support

- **e-con Systems:** https://www.e-consystems.com/
- **NVIDIA Jetson Forums:** https://forums.developer.nvidia.com/
- **Build Guide:** See BUILD_ECAM25_JP621_GUIDE.md in the ecam-25docs folder

---

**Note:** This is a community build based on e-con Systems' driver package. For official support, contact e-con Systems directly.
