# e-CAM25_CUONX Camera Setup for Jetson Orin Nano

**Camera:** e-CAM25_CUONX (AR0234 global shutter, 2.3MP)  
**Interface:** MIPI CSI-2  
**JetPack:** 6.2 (L4T 36.4.3) or 6.1 (L4T 36.4.0)

---

## ⚠️ Important: Custom Drivers Required

The e-CAM25_CUONX is an **ISP camera** that requires:
- Custom kernel module (`ar0234.ko`)
- Device Tree Blob Overlay (DTBO)
- ISP tuning files (`camera_overrides.isp`)
- e-con's argus_camera application

**You CANNOT use generic v4l2_camera** - it won't work!

---

## 📦 Prerequisites

### 1. e-CAM25_CUONX Release Package

**Location:** `/home/ubuntulaptop/Tank_projects/ecam-25docs/`

**Package:** `e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_29-MAR-2025_R04/`

**Contents:**
- Kernel drivers (ar0234.ko, tegra-camera.ko)
- DTBO files (2-lane and 4-lane configs)
- Application binaries (eCAM_argus_camera)
- ISP tuning files
- Firmware (if needed)
- `install_binaries.sh` - automated installer

---

## 🔧 Installation (On Jetson)

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

## 🔄 Upgrading to JetPack 6.2

If you need JetPack 6.2 (L4T 36.4.3), follow the upgrade guide in:
- `/home/ubuntulaptop/Tank_projects/ecam-25docs/econ25_upgradeto6.2.txt`

**Quick upgrade steps:**
1. Flash Jetson with JetPack 6.2 using SDK Manager
2. Follow "Upgrading from JetPack 6.1 to JetPack 6.2" section
3. Apply kernel patches from e-con release package
4. Rebuild and install drivers

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

