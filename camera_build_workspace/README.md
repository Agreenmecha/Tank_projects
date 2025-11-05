# Camera Build Workspace

This directory contains everything needed to build the e-CAM25_CUONX camera drivers for Jetson Orin Nano with JetPack 6.2.1.

## Directory Structure

```
camera_build_workspace/
├── README.md                    # This file
├── docs/                        # Build documentation
│   ├── BUILD_GUIDE_NATIVE_PC.md      # Complete build instructions
│   ├── QUICK_START.txt               # Quick reference
│   └── TROUBLESHOOTING.md            # Common issues
├── scripts/                     # Build automation scripts
│   ├── build_drivers.sh              # Main build script for Linux PC
│   └── install_on_jetson.sh          # Installation script for Jetson
├── Linux_for_Tegra/            # NVIDIA L4T (extracted during build)
├── sensor_driver/              # Camera driver source (created during build)
└── jetson_binaries/            # Output directory (ready to copy to Jetson)
```

## Required Files (Not in Git - Too Large)

These files must be downloaded/obtained separately:

### From NVIDIA (Already in ../nvidiaR36.4.3/)
- `Jetson_Linux_R36.4.3_aarch64.tbz2` (683MB)
- `Tegra_Linux_Sample-Root-Filesystem_R36.4.3_aarch64.tbz2` (1.8GB)
- `public_sources.tbz2` (216MB)

### Download Separately
- `aarch64--glibc--stable-2022.08-1.tar.bz2` (138MB)
  - From: https://toolchains.bootlin.com/downloads/releases/toolchains/aarch64/tarballs/

### From e-con Systems (Should be in repo)
- `e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_29-MAR-2025_R04/`
  - Camera patches, firmware, and ISP tuning files

## Quick Start

### On Native Linux PC (Ubuntu 20.04/22.04):

```bash
cd /home/agreen/Tank_projects/camera_build_workspace
./scripts/build_drivers.sh
```

Wait 30-60 minutes for build to complete.

### Transfer to Jetson:

```bash
cd /home/agreen/Tank_projects/camera_build_workspace/jetson_binaries
scp * aaron@jetson-ip:~/
```

### Install on Jetson:

```bash
cd ~
bash install_on_jetson.sh
sudo reboot
```

## Documentation

Read the complete guide: `docs/BUILD_GUIDE_NATIVE_PC.md`

## Build Requirements

- **OS:** Ubuntu 20.04/22.04 (native, not WSL)
- **Disk:** 50GB free space
- **RAM:** 8GB minimum, 16GB recommended
- **Time:** 30-60 minutes

## Important Notes

1. **Build on Native Linux PC Only** - WSL won't work for kernel module compilation
2. **Use L4T 36.4.3 sources** - Compatible with JetPack 6.2.1 (L4T 36.4.7) on Jetson
3. **4-lane configuration recommended** - For 1280x720 @ 20fps performance

## Status

Check `../PROJECT_STATUS.md` for overall project status and camera integration with ROS2.

