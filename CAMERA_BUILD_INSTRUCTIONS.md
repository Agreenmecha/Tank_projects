# Camera Driver Build Instructions

**For e-CAM25_CUONX (AR0234) on Jetson Orin Nano with JetPack 6.2.1**

## Quick Overview

This repository contains everything needed to build and install the e-CAM25_CUONX camera drivers for the Tank project.

**Important:** Camera drivers must be built on a **native Linux PC** (Ubuntu 20.04/22.04), not in WSL or on the Jetson itself.

## Directory Structure

```
Tank_projects/
├── camera_build_workspace/          ← Main build workspace
│   ├── docs/
│   │   ├── BUILD_GUIDE_NATIVE_PC.md ← Complete instructions
│   │   └── QUICK_START.txt          ← Quick reference
│   ├── scripts/
│   │   └── build_drivers.sh         ← Automated build script
│   └── README.md
├── nvidiaR36.4.3/                  ← NVIDIA downloads (not in git)
├── e-CAM25_CUONX_.../              ← Camera patches & firmware
└── drivers/                         ← Pre-built drivers (fallback)
```

## Quick Start

### Prerequisites

- **Native Linux PC** with Ubuntu 20.04 or 22.04
- 50GB free disk space
- 8GB RAM (16GB recommended)
- Internet connection for downloads

### Steps

1. **Clone this repository on your Linux PC:**
   ```bash
   cd ~
   git clone https://github.com/YOUR_USERNAME/Tank_projects.git
   cd Tank_projects/camera_build_workspace
   ```

2. **Install build tools:**
   ```bash
   sudo apt update
   sudo apt install -y build-essential bc bison flex libssl-dev \
       libncurses-dev git wget curl tar bzip2 lbzip2 \
       python3 python3-pip device-tree-compiler
   ```

3. **Download Bootlin toolchain:**
   ```bash
   wget https://toolchains.bootlin.com/downloads/releases/toolchains/aarch64/tarballs/aarch64--glibc--stable-2022.08-1.tar.bz2
   ```

4. **Download NVIDIA files** (if not already in repo):
   - Go to: https://developer.nvidia.com/embedded/jetpack
   - Download L4T 36.4.3 packages (see detailed guide)
   - Place in `../nvidiaR36.4.3/` directory

5. **Run the build:**
   ```bash
   ./scripts/build_drivers.sh
   ```
   Wait 30-60 minutes for build to complete.

6. **Transfer to Jetson:**
   ```bash
   cd jetson_binaries
   scp * aaron@<jetson-ip>:~/
   ```

7. **Install on Jetson:**
   ```bash
   # On Jetson
   cd ~
   bash install_on_jetson.sh
   sudo reboot
   ```

## Documentation

- **📖 Complete Build Guide:** `camera_build_workspace/docs/BUILD_GUIDE_NATIVE_PC.md`
- **⚡ Quick Start:** `camera_build_workspace/docs/QUICK_START.txt`
- **🔧 Troubleshooting:** `camera_build_workspace/docs/BUILD_GUIDE_NATIVE_PC.md#troubleshooting`

## Important Notes

### Why Native Linux PC?

NVIDIA kernel module compilation requires:
- Native Linux kernel headers
- Direct hardware access
- Tools not available in WSL

### Why L4T 36.4.3 Sources for JetPack 6.2.1?

The Jetson runs JetPack 6.2.1 (L4T 36.4.7), but we build with L4T 36.4.3 sources because:
- Kernel version is the same (5.15.148-tegra)
- L4T 36.4.3 sources are officially supported
- Modules are compatible across minor L4T versions

### What Gets Built?

- **Kernel modules:**
  - `ar0234.ko` - Camera sensor driver
  - `tegra-camera.ko` - Camera host driver
  - `capture-ivc.ko` - Capture IVC driver
  
- **Device tree overlay:**
  - `tegra234-p3767-0000-p3768-0000-a0-4lane-ar0234.dtbo`
  
- **ISP tuning file:**
  - `camera_overrides.isp`

## What's In Git vs. What to Download

### ✅ Committed to Git

- Documentation
- Build scripts
- e-CAM25 patches and firmware
- ROS2 workspace
- Configuration files

### ❌ NOT in Git (too large)

- NVIDIA L4T packages (2.6GB)
- Bootlin toolchain (138MB)
- Extracted source trees (30GB+)
- Build artifacts

These are excluded via `.gitignore` and must be downloaded separately.

## Verification

After installation on Jetson:

```bash
# Check camera device
ls /dev/video0

# Check driver loaded
lsmod | grep ar0234

# Test with v4l2
v4l2-ctl --list-devices

# Test with GStreamer
gst-launch-1.0 v4l2src device=/dev/video0 ! \
    'video/x-raw,width=1280,height=720,framerate=20/1' ! \
    xvimagesink sync=false
```

## ROS2 Integration

After successful camera installation, integrate with ROS2:

```bash
cd ~/Tank_projects/tank_ws
source install/setup.bash
ros2 launch gscam gscam.launch.py
```

See `PHASE1_IMPLEMENTATION.md` for complete ROS2 setup.

## Support

If you encounter issues:

1. Read the complete guide: `camera_build_workspace/docs/BUILD_GUIDE_NATIVE_PC.md`
2. Check troubleshooting section
3. Verify you're on native Linux (not WSL)
4. Check disk space: `df -h`
5. Review build logs: `camera_build_workspace/build.log`

## Related Documentation

- `PROJECT_STATUS.md` - Overall project status
- `ECAM25_BUILD_OVERVIEW.md` - Build process overview
- `ecam-25docs/` - e-con Systems documentation
- `PHASE1_IMPLEMENTATION.md` - Tank project Phase 1 implementation

---

**Ready to build?** Follow the quick start above or read the complete guide in `camera_build_workspace/docs/BUILD_GUIDE_NATIVE_PC.md`

