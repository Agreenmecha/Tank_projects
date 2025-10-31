# e-CAM25_CUONX Camera Setup for Jetson Orin Nano

**Camera:** e-CAM25_CUONX (AR0234 global shutter, 2.3MP)  
**Interface:** MIPI CSI-2  
**Supported JetPack Versions:**
- **JetPack 6.2** (L4T 36.4.3) - **RECOMMENDED** ✅
- JetPack 6.1 (L4T 36.4.0) - Supported

---

## ⚠️ Important: Custom Drivers Required

The e-CAM25_CUONX is an **ISP camera** that requires:
- Custom kernel module (`ar0234.ko`)
- Device Tree Blob Overlay (DTBO)
- ISP tuning files (`camera_overrides.isp`)
- e-con's argus_camera application

**You CANNOT use generic v4l2_camera** - it won't work!

---

## 🎯 Quick Start - Choose Your Path

### Path A: Fresh JetPack 6.2 Installation (Recommended)
If you're flashing Jetson with JetPack 6.2 from scratch, follow **Section 1: JetPack 6.2 Fresh Installation**.

### Path B: Upgrading from JetPack 6.1 to 6.2
If you already have JetPack 6.1 installed, follow **Section 2: Upgrade from 6.1 to 6.2**.

### Path C: Using JetPack 6.1
If staying on JetPack 6.1, follow **Section 3: JetPack 6.1 Installation**.

---

## 📦 Prerequisites (All Paths)

### e-CAM25_CUONX Release Package

**Location on Laptop:** `/home/ubuntulaptop/Tank_projects/ecam-25docs/`

**Available Packages:**
- **For JetPack 6.1:** `e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_29-MAR-2025_R04/`
- **For JetPack 6.2:** You'll need to request from e-con Systems OR build from 6.1 package (instructions below)

**Package Contents:**
- Kernel drivers (ar0234.ko, tegra-camera.ko, capture-ivc.ko)
- DTBO files (2-lane and 4-lane configs)
- Application binaries (eCAM_argus_camera)
- ISP tuning files
- Firmware
- Kernel patches for driver building

---

## 1️⃣ JetPack 6.2 Fresh Installation (L4T 36.4.3)

### Overview
JetPack 6.2 brings **Super Mode** with enhanced power management and performance. Installing the camera requires building drivers from source patches since the pre-built package is for 6.1.

### Step 1: Flash Jetson with JetPack 6.2

**Option A: Using NVIDIA SDK Manager (Recommended)**
```bash
# On your Linux PC:
# 1. Download SDK Manager from NVIDIA: https://developer.nvidia.com/sdk-manager
# 2. Install and run SDK Manager
# 3. Select:
#    - Hardware: Jetson Orin Nano Developer Kit
#    - JetPack: 6.2 (L4T 36.4.3)
#    - Components: Jetson OS, SDK Components
# 4. Follow prompts to flash the device
```

**Option B: Using Command Line**
```bash
# Download L4T 36.4.3 from NVIDIA
# See: https://developer.nvidia.com/embedded/jetson-linux-archive
```

### Step 2: Prepare Build Environment on Linux PC

You need a **Linux PC** (Ubuntu 20.04/22.04) to build the camera drivers.

```bash
# On your Linux development PC, create workspace
mkdir -p ~/jetson_camera_build
cd ~/jetson_camera_build

# Set environment variables
export TOP_DIR=$PWD
export RELEASE_PACK_DIR=$PWD/econ_release
export NVIDIA_SRC=$PWD/Linux_for_Tegra/source
export LDK_ROOTFS_DIR=$PWD/Linux_for_Tegra/rootfs
export MAKEFILE_DIR=$PWD/sensor_driver
```

### Step 3: Download Requirements

```bash
cd ~/jetson_camera_build

# 1. Download Bootlin Toolchain
wget https://toolchains.bootlin.com/downloads/releases/toolchains/aarch64/tarballs/aarch64--glibc--stable-2020.08-1.tar.bz2
tar -xf aarch64--glibc--stable-2020.08-1.tar.bz2

# 2. Download NVIDIA JetPack 6.2 sources
# Driver Package (BSP)
wget https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v4.3/release/jetson_linux_r36.4.3_aarch64.tbz2

# Sample Root Filesystem
wget https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v4.3/release/tegra_linux_sample-root-filesystem_r36.4.3_aarch64.tbz2

# Driver Package Sources
wget https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v4.3/sources/public_sources.tbz2

# 3. Copy e-con release package from laptop
cp -r /home/ubuntulaptop/Tank_projects/ecam-25docs/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_29-MAR-2025_R04 $RELEASE_PACK_DIR
```

### Step 4: Extract and Setup L4T

```bash
cd ~/jetson_camera_build

# Extract driver package
sudo tar -xpf jetson_linux_r36.4.3_aarch64.tbz2

# Extract rootfs
cd Linux_for_Tegra/rootfs
sudo tar -xpf ../../tegra_linux_sample-root-filesystem_r36.4.3_aarch64.tbz2

# Apply NVIDIA binaries
cd ..
sudo ./apply_binaries.sh

# Extract kernel sources
cd ~jetson_camera_build
tar -xf public_sources.tbz2
cd Linux_for_Tegra/source
tar -xf kernel_src.tbz2
tar -xf kernel_oot_modules_src.tbz2

# Setup toolchain
export CROSS_COMPILE=~/jetson_camera_build/aarch64--glibc--stable-2020.08-1/bin/aarch64-linux-
export KERNEL_HEADERS=$NVIDIA_SRC/kernel/kernel-jammy-src
export NVIDIA_HEADERS=$NVIDIA_SRC/nvidia-oot
export NVIDIA_CONFTEST=$NVIDIA_HEADERS/conftest
```

### Step 5: Configure Kernel for Camera

```bash
cd $NVIDIA_SRC/kernel/kernel-jammy-src

# Create sensor driver directory
mkdir -p ~/jetson_camera_build/sensor_driver

# Apply e-con module patch
patch -d ~/jetson_camera_build/sensor_driver -p1 -i $RELEASE_PACK_DIR/Kernel/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_module.patch

# Remove vcc-supply from device tree patch
sed -i 's/vcc-supply/\/\/vcc-supply/g' $RELEASE_PACK_DIR/Kernel/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_dtb.patch

# Apply device tree overlay patch
patch -p1 -i $RELEASE_PACK_DIR/Kernel/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_dtb.patch

# If you get a "Reversed patch" message, enter 'n' (no)

# Remove g_parm and s_parm functions from nvidia-oot patch (for e-CAM25_CUONX)
sed '/@@ -2281,6 +2285,28 @@ static long tegra_channel_default_ioctl/,/tegra_channel_close/d' \
    $RELEASE_PACK_DIR/Kernel/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_oot.patch > /tmp/tempfile && \
    mv /tmp/tempfile $RELEASE_PACK_DIR/Kernel/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_oot.patch

# Apply nvidia-oot patch
patch -p1 -i $RELEASE_PACK_DIR/Kernel/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_oot.patch

# Fix vi5_fops.c for proper camera streaming
sed '238,238d' $NVIDIA_SRC/nvidia-oot/drivers/media/platform/tegra/camera/vi/vi5_fops.c > /tmp/tempfile && \
    mv /tmp/tempfile $NVIDIA_SRC/nvidia-oot/drivers/media/platform/tegra/camera/vi/vi5_fops.c
sed '228i#endif' $NVIDIA_SRC/nvidia-oot/drivers/media/platform/tegra/camera/vi/vi5_fops.c > /tmp/tempfile && \
    mv /tmp/tempfile $NVIDIA_SRC/nvidia-oot/drivers/media/platform/tegra/camera/vi/vi5_fops.c
```

### Step 6: Build Kernel and Modules

```bash
cd ~/jetson_camera_build/Linux_for_Tegra/source

# Build kernel image
make -C kernel

# Build modules
make modules

# Install modules to rootfs
sudo -E make modules_install

# Build device tree blobs
make dtbs
```

### Step 7: Deploy to Jetson

**Option A: Copy binaries to running Jetson (Easier)**

```bash
# On development PC, connect Jetson via ethernet/USB
# Jetson should be running stock JetPack 6.2

# 1. Copy DTBO
cd $NVIDIA_SRC/kernel-devicetree/generic-dts/dtbs/
scp tegra234-p3767-0000-p3768-0000-a0-4lane-ar0234.dtbo nvidia@<jetson-ip>:~/

# 2. Compress and copy kernel modules
cd $LDK_ROOTFS_DIR/lib/
sudo tar -cjmpf kernel_supplements.tar.bz2 modules/
scp kernel_supplements.tar.bz2 nvidia@<jetson-ip>:~/

# 3. Copy firmware (if exists)
# cd $TOP_DIR
# scp -r $RELEASE_PACK_DIR/Firmware nvidia@<jetson-ip>:~/

# 4. Copy ISP tuning file
cd $TOP_DIR
scp $RELEASE_PACK_DIR/misc/camera_overrides_jetson-onano.isp nvidia@<jetson-ip>:~/
```

**On Jetson, install binaries:**

```bash
# Install DTBO
sudo cp tegra234-p3767-0000-p3768-0000-a0-4lane-ar0234.dtbo /boot/

# Install kernel modules
sudo tar -xjmpf kernel_supplements.tar.bz2 -C /usr/lib/

# Install firmware (if exists)
# sudo cp Firmware/* /lib/firmware/

# Install ISP tuning file
sudo cp camera_overrides_jetson-onano.isp /var/nvidia/nvcam/settings/camera_overrides.isp
sudo chmod 664 /var/nvidia/nvcam/settings/camera_overrides.isp
sudo chown root:root /var/nvidia/nvcam/settings/camera_overrides.isp

# Load overlay
sudo /opt/nvidia/jetson-io/config-by-hardware.py -n2="e-CAM25 CUONX AR0234"

# Reboot
sudo reboot
```

### Step 8: Verify Installation

```bash
# After reboot, check camera detection
ls /dev/video*
# Should show: /dev/video0

# Check ar0234 module
lsmod | grep ar0234

# Test camera
v4l2-ctl --list-devices
```

---

## 2️⃣ Upgrade from JetPack 6.1 to 6.2

If you already have JetPack 6.1 with camera working, upgrading to 6.2 is simpler.

### Prerequisites
- Jetson running JetPack 6.1 with e-CAM25 already working
- e-con release package for JetPack 6.1

### Step 1: Flash to JetPack 6.2

Use NVIDIA SDK Manager to flash JetPack 6.2 to your Jetson.

### Step 2: Follow Simplified Build Process

The process is similar to Section 1, but you'll use the 6.1 patches which are mostly compatible. Follow Steps 2-8 from Section 1 above.

**Key Differences:**
- The nvidia-oot patch removal command is slightly different (already shown above)
- All other steps are identical

---

## 3️⃣ JetPack 6.1 Installation (L4T 36.4.0)

### Simple Installation with Pre-built Package

This is the easiest path since e-con provides pre-built binaries for JetPack 6.1.

---

## 🔧 Installation (JetPack 6.1 with Pre-built Package)

### Step 1: Copy Release Package to Jetson

```bash
# From your laptop, copy the release package to Jetson
cd /home/ubuntulaptop/Tank_projects/ecam-25docs/
scp -r e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_29-MAR-2025_R04 nvidia@<jetson-ip>:~/
```

### Step 2: Run Installation Script

```bash
# On Jetson
cd ~/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_29-MAR-2025_R04
sudo ./install_binaries.sh

# You will be prompted for lane configuration:
# Enter: 1 for 2-lane OR 2 for 4-lane
# Recommended: 4-lane for better performance (70fps @ Full HD)
```

**The script will:**
1. Check L4T version matches (36.4.0 for JetPack 6.1)
2. Extract release package
3. Install kernel drivers (ar0234.ko)
4. Install DTBO file
5. Configure overlay with jetson-io
6. Install eCAM_argus_camera application
7. Copy ISP tuning files
8. **Reboot automatically**

### Step 3: Verify Installation (After Reboot)

```bash
# Check if camera is detected
ls /dev/video*
# Should show: /dev/video0

# Check loaded modules
lsmod | grep ar0234
# Should show: ar0234 module loaded

# Check camera node
v4l2-ctl --list-devices
# Should show: vi-output device
```

---

## 🎥 Testing the Camera

### Using eCAM_argus_camera (Recommended)

```bash
# Capture and display
eCAM_argus_camera --capture

# Save to file
eCAM_argus_camera --capture --savefile

# Check supported modes
eCAM_argus_camera --mode-list
```

### Using GStreamer (for ROS integration)

```bash
# Test camera streaming
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! \
    'video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=20/1' ! \
    nvvidconv ! 'video/x-raw, format=BGRx' ! \
    videoconvert ! 'video/x-raw, format=BGR' ! \
    autovideosink

# Check camera properties
v4l2-ctl --all --device /dev/video0
```

### Expected Output (Camera Specs)

```
Format: YUYV/UYVY (uncompressed)
Resolution: 1920x1200 (2.3MP max)
Frame Rates:
  - 1280x720 @ 120fps (HD)
  - 1920x1080 @ 70fps (Full HD)
  - 1920x1200 @ 60fps (2.3MP)
Shutter: Global
FOV: 128.2°(D), 104.6°(H), 61.6°(V)
```

---

## 📊 Installation Summary & Comparison

### JetPack 6.2 vs 6.1

| Feature | JetPack 6.2 | JetPack 6.1 |
|---------|-------------|-------------|
| **Installation Time** | 2-3 hours (build from source) | 5-10 minutes (pre-built) |
| **Complexity** | High (requires Linux PC + cross-compile) | Low (automated script) |
| **Performance** | ⭐⭐⭐⭐⭐ Super Mode enabled | ⭐⭐⭐ Standard modes |
| **Power Management** | Enhanced (25W/MAXN SUPER modes) | Standard power modes |
| **Frame Rate** | Higher (up to 70fps @ 1080p) | Standard (up to 70fps @ 1080p) |
| **AI Inference** | Faster (lower latency) | Standard |
| **Stability** | New (less tested) | Stable (well-tested) |

### Recommended for Tank Project: **JetPack 6.2** ✅

**Why JetPack 6.2 is better for your tank:**

1. **Isaac ROS Performance:** Super Mode provides more compute headroom for DNN inference (camera segmentation)
2. **Multi-tasking:** Better power management for running Point-LIO + perception + navigation simultaneously
3. **Future-proof:** Latest features and optimization from NVIDIA
4. **Real-time Performance:** Lower latency critical for obstacle avoidance on rough terrain

**The 2-3 hour build time is a one-time cost** that pays off in production performance!

### Quick Reference: What You Need

**For JetPack 6.2:**
- Linux PC (Ubuntu 20.04/22.04) for building drivers
- NVIDIA SDK Manager
- e-con release package (JetPack 6.1 patches work for 6.2)
- 2-3 hours for initial setup
- See **Section 1** above for detailed steps

**For JetPack 6.1:**
- Just the Jetson and e-con release package
- 10 minutes for installation
- See **Section 3** above for detailed steps

---

## 🎯 ROS2 Integration

### Option A: Using gscam (ROS2 GStreamer camera)

**Install gscam:**
```bash
sudo apt install ros-humble-gscam
```

**Launch file:** `tank_sensors/launch/camera_argus.launch.py`

```python
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gscam',
            executable='gscam_node',
            name='camera',
            output='screen',
            parameters=[{
                'camera_name': 'ecam25_cuonx',
                'camera_info_url': '',
                'frame_id': 'camera_link',
                'gscam_config': 'nvarguscamerasrc sensor-id=0 ! '
                                'video/x-raw(memory:NVMM), width=1280, height=720, '
                                'format=NV12, framerate=20/1 ! '
                                'nvvidconv ! video/x-raw, format=BGRx ! '
                                'videoconvert ! video/x-raw, format=BGR',
                'sync_sink': True,
                'use_gst_timestamps': False,
            }],
            remappings=[
                ('camera/image_raw', '/camera/image_raw'),
                ('camera/camera_info', '/camera/camera_info'),
            ]
        )
    ])
```

### Option B: Custom argus_camera ROS2 node (Best Performance)

For optimal performance, create a custom ROS2 node that uses NVIDIA Argus API directly. This avoids GStreamer overhead and provides:
- Lower latency (~30ms vs ~100ms)
- Direct CUDA memory access
- Better integration with Isaac ROS

**TODO:** Implement `tank_sensors/src/argus_camera_node.cpp`

---

## 📊 Camera Configuration for Tank

**Recommended Settings:**
- **Resolution:** 1280x720 (good balance of quality vs compute)
- **Frame Rate:** 20 fps (matches plan, reduces CPU load)
- **Format:** UYVY (native) → convert to BGR for Isaac ROS
- **Exposure:** Auto-exposure for outdoor use
- **Global Shutter:** Eliminates motion blur on rough terrain

**Mount Position (from plan):**
- Height: ~35cm AGL
- Tilt: -15° (downward)
- Position: Front center
- FOV: 104.6° horizontal (good coverage)

---

## 🐛 Troubleshooting

### Problem: Camera not detected

**Solution:**
```bash
# Check if DTBO is loaded
sudo /opt/nvidia/jetson-io/jetson-io.py
# Verify e-CAM25_CUONX overlay is active

# Check device tree
ls /proc/device-tree/ | grep ar0234
# Should show ar0234 entries

# Reload camera module
sudo modprobe -r ar0234
sudo modprobe ar0234
```

### Problem: "No cameras available" error

**Solution:**
```bash
# Check ISP tuning file
ls -l /var/nvidia/nvcam/settings/camera_overrides.isp
# Should exist with 664 permissions

# Check nvargus-daemon
sudo systemctl status nvargus-daemon
sudo systemctl restart nvargus-daemon
```

### Problem: Black image or no output

**Solution:**
```bash
# Check CSI connection - ensure flex cable is properly seated
# Run ISP max clocks script
~/max-isp-vi-clks.sh

# Check if camera is getting power
# Verify 3.3V on camera module
```

### Problem: Low frame rate or stuttering

**Solution:**
```bash
# Enable max performance mode
sudo nvpmodel -m 0  # MAXN mode
sudo jetson_clocks

# Check if ISP clocks are maxed
~/max-isp-vi-clks.sh
```

---

## 🔗 References

**e-con Systems Documentation:**
- Developer Guide: `e-CAM25_CUONX_Developer_Guide_Rev_1_6.pdf`
- GStreamer Guide: `e-CAM25_CUONX_GStreamer_Usage_Guide_Rev_1_4.pdf`
- Release Notes: `e-CAM25_CUONX_Release_Notes_Rev_1_7.pdf`

**NVIDIA Documentation:**
- Argus API: https://docs.nvidia.com/jetson/l4t-multimedia/group__LibargusAPI.html
- Camera Driver Development: https://docs.nvidia.com/jetson/l4t/Camera+Development.html

**Project Files:**
- Camera docs: `/home/ubuntulaptop/Tank_projects/ecam-25docs/`
- Installation package: `e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_29-MAR-2025_R04/`

---

## ✅ Installation Checklist

- [ ] Copy release package to Jetson
- [ ] Run `sudo ./install_binaries.sh`
- [ ] Select 4-lane configuration
- [ ] Wait for automatic reboot
- [ ] Verify camera detection (`ls /dev/video0`)
- [ ] Test with `eCAM_argus_camera --capture`
- [ ] Test GStreamer pipeline
- [ ] Install ros-humble-gscam
- [ ] Test ROS2 camera node
- [ ] Verify 20fps output

---

**Note:** The e-CAM25_CUONX is a professional-grade camera with excellent global shutter performance. Proper installation of e-con's drivers is essential for optimal performance on rough terrain!

