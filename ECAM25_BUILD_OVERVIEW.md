# e-CAM25_CUONX Driver Build Overview for JetPack 6.2.1

**Date:** November 4, 2025  
**Your System:** JetPack 6.2.1 (L4T 36.4.7)  
**Camera:** e-CAM25_CUONX (AR0234 global shutter)

---

## ⚠️ Critical Requirement: Build on Linux PC

**You CANNOT build the drivers on the Jetson!**  
The camera drivers must be built on a **Linux PC** (Ubuntu 20.04/22.04) and then copied to the Jetson.

**Why?**
- Requires cross-compilation toolchain (aarch64)
- Needs NVIDIA kernel sources (large download ~2GB)
- Build process is complex and requires Linux PC environment

---

## 📋 Build Process Summary

### **What Needs to Be Built:**

1. **Kernel Modules:**
   - `ar0234.ko` - Camera sensor driver
   - `tegra-camera.ko` - Modified camera platform driver
   - `capture-ivc.ko` - Camera capture driver

2. **Device Tree Overlay (DTBO):**
   - `tegra234-p3767-0000-p3768-0000-a0-4lane-ar0234.dtbo` (4-lane, recommended)
   - OR `tegra234-p3767-0000-p3768-0000-a0-2lane-ar0234.dtbo` (2-lane)

3. **Kernel Image:**
   - Modified kernel with camera patches applied

---

## 🎯 Why Build for 6.2.x Even Though You Have 6.2.1?

**Answer:** JetPack 6.2.1 (L4T 36.4.7) uses the **same kernel sources** as JetPack 6.2 (L4T 36.4.3).

- Both use Linux Kernel 5.15
- Both use the same device tree structure
- Binaries built from 6.2 sources work on 6.2.1

**Table from e-con upgrade guide:**

| JetPack Version | L4T Version | Kernel | Sources to Download |
|----------------|-------------|--------|-------------------|
| 6.2 | 36.4.3 | 5.15 | `Jetson_Linux_R36.4.3_aarch64.tbz2` |
| 6.2.1 | 36.4.7 | 5.15 | **Same as 6.2** (36.4.3) |

**Conclusion:** Download L4T 36.4.3 sources even though you're on 6.2.1!

---

## 📦 Required Downloads (on Linux PC)

### **1. Bootlin Cross-Compilation Toolchain**
```
File: aarch64--glibc--stable-2022.08-1.tar.bz2
URL: https://toolchains.bootlin.com/downloads/releases/toolchains/aarch64/tarballs/aarch64--glibc--stable-2022.08-1.tar.bz2
Size: ~100MB
```

### **2. NVIDIA JetPack 6.2 Sources (L4T 36.4.3)**

**Note:** Even though you're on 6.2.1, download 6.2 (36.4.3) sources!

| File | URL | Size |
|------|-----|------|
| **Driver Package (BSP)** | `Jetson_Linux_R36.4.3_aarch64.tbz2` | ~1.5GB |
| **Sample Root Filesystem** | `Tegra_Linux_Sample-Root-Filesystem_R36.4.3_aarch64.tbz2` | ~3GB |
| **Driver Package Sources** | `public_sources.tbz2` | ~2GB |

**Where to download:**
- https://developer.nvidia.com/embedded/jetpack
- Log in with NVIDIA Developer account
- Navigate to JetPack 6.2 → L4T 36.4.3

### **3. e-CAM25 Release Package**

**You already have this!**
```
Location: ~/Tank_projects/ecam-25docs/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_29-MAR-2025_R04/
```

**Package Contents:**
- `Kernel/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_module.patch`
- `Kernel/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_dtb.patch`
- `Kernel/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_oot.patch`
- `misc/camera_overrides_jetson-onano.isp` (ISP tuning)
- `Firmware/ecam25_cuonx_mcu_fw.bin` (MCU firmware)

---

## 🔧 Build Steps Overview

### **On Linux PC:**

#### **Step 1: Setup Environment**
```bash
mkdir -p ~/jetson_camera_build
cd ~/jetson_camera_build

# Set environment variables
export TOP_DIR=$PWD
export RELEASE_PACK_DIR=$PWD/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_29-MAR-2025_R04
export NVIDIA_SRC=$PWD/Linux_for_Tegra/source
export CROSS_COMPILE=$PWD/aarch64--glibc--stable-2022.08-1/bin/aarch64-buildroot-linux-gnu-
export KERNEL_HEADERS=$NVIDIA_SRC/kernel/kernel-jammy-src
export NVIDIA_HEADERS=$NVIDIA_SRC/nvidia-oot
```

#### **Step 2: Download & Extract Sources**
- Download Bootlin toolchain
- Download NVIDIA L4T 36.4.3 sources
- Extract all packages
- Copy e-CAM25 release package to workspace

#### **Step 3: Apply Kernel Patches**

**Apply patches in this order:**

1. **Module Patch** (creates sensor driver directory):
   ```bash
   patch -d sensor_driver -p1 -i $RELEASE_PACK_DIR/Kernel/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_module.patch
   ```

2. **Device Tree Patch** (modify for 6.2.x first):
   ```bash
   # Remove vcc-supply (not needed for 6.2.x)
   sed -i 's/vcc-supply/\/\/vcc-supply/g' $RELEASE_PACK_DIR/Kernel/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_dtb.patch
   
   # Apply patch
   patch -p1 -i $RELEASE_PACK_DIR/Kernel/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_dtb.patch
   ```

3. **nvidia-oot Patch** (modify first):
   ```bash
   # For e-CAM25_CUONX: Remove g_parm and s_parm functions
   sed '/@@ -2281,6 +2285,28 @@ static long tegra_channel_default_ioctl/,/tegra_channel_close/d' \
       $RELEASE_PACK_DIR/Kernel/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_oot.patch \
       > /tmp/tempfile && \
       mv /tmp/tempfile $RELEASE_PACK_DIR/Kernel/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_oot.patch
   
   # Apply patch
   patch -p1 -i $RELEASE_PACK_DIR/Kernel/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_oot.patch
   ```

4. **Fix vi5_fops.c** (for proper camera streaming):
   ```bash
   sed '238,238d' $NVIDIA_SRC/nvidia-oot/drivers/media/platform/tegra/camera/vi/vi5_fops.c > /tmp/tempfile && \
       mv /tmp/tempfile $NVIDIA_SRC/nvidia-oot/drivers/media/platform/tegra/camera/vi/vi5_fops.c
   sed '228i#endif' $NVIDIA_SRC/nvidia-oot/drivers/media/platform/tegra/camera/vi/vi5_fops.c > /tmp/tempfile && \
       mv /tmp/tempfile $NVIDIA_SRC/nvidia-oot/drivers/media/platform/tegra/camera/vi/vi5_fops.c
   ```

#### **Step 4: Build**
```bash
cd $NVIDIA_SRC

# Build kernel
make -C kernel

# Build modules (takes 30-60 minutes)
make modules

# Install modules to rootfs
sudo -E make modules_install

# Build device tree blobs (DTBO)
make dtbs
```

**Output Files:**
- Kernel modules in `Linux_for_Tegra/rootfs/lib/modules/`
- DTBO files in `Linux_for_Tegra/source/kernel-devicetree/generic-dts/dtbs/`

#### **Step 5: Copy to Jetson**
```bash
# Copy DTBO
scp tegra234-p3767-0000-p3768-0000-a0-4lane-ar0234.dtbo aaron@<jetson-ip>:~/

# Compress and copy modules
cd Linux_for_Tegra/rootfs/lib/
sudo tar -cjmpf kernel_supplements.tar.bz2 modules/
scp kernel_supplements.tar.bz2 aaron@<jetson-ip>:~/

# Copy ISP tuning file
scp $RELEASE_PACK_DIR/misc/camera_overrides_jetson-onano.isp aaron@<jetson-ip>:~/
```

### **On Jetson:**

#### **Step 6: Install Drivers**
```bash
# Install DTBO
sudo cp ~/tegra234-p3767-0000-p3768-0000-a0-4lane-ar0234.dtbo /boot/

# Install kernel modules
sudo tar -xjmpf ~/kernel_supplements.tar.bz2 -C /usr/lib/

# Update module dependencies
sudo depmod -a

# Install ISP tuning file
sudo cp ~/camera_overrides_jetson-onano.isp /var/nvidia/nvcam/settings/camera_overrides.isp
sudo chmod 664 /var/nvidia/nvcam/settings/camera_overrides.isp
```

#### **Step 7: Configure Device Tree Overlay**
```bash
# Backup extlinux.conf
sudo cp /boot/extlinux/extlinux.conf /boot/extlinux/extlinux.conf.backup_$(date +%Y%m%d_%H%M%S)

# Edit extlinux.conf
sudo nano /boot/extlinux/extlinux.conf
```

**Add this line after the FDT line:**
```
      OVERLAYS /boot/tegra234-p3767-0000-p3768-0000-a0-4lane-ar0234.dtbo
```

#### **Step 8: Reboot and Test**
```bash
sudo reboot
```

**After reboot, verify:**
```bash
# Check camera device
ls /dev/video*

# Check modules loaded
lsmod | grep ar0234

# Test camera
v4l2-ctl --list-devices
```

---

## 🔍 Key Differences for JetPack 6.2.x

### **1. Patch Modifications Required:**

**Device Tree Patch:**
- Remove `vcc-supply` references (not supported in 6.2.x)
- Command: `sed -i 's/vcc-supply/\/\/vcc-supply/g' <patch_file>`

**nvidia-oot Patch:**
- Remove `g_parm` and `s_parm` functions (different line numbers for e-CAM25_CUONX)
- For e-CAM25_CUONX: Line 2281, not 2224

**vi5_fops.c Fix:**
- Line numbers are different (238 vs 236 vs 240)
- For e-CAM25_CUONX: Use lines 238 and 228

### **2. Lane Configuration:**

**Choose based on your needs:**

| Configuration | Max FPS | Max Resolution | Bandwidth |
|--------------|---------|----------------|-----------|
| **4-lane (Recommended)** | 70fps | 1920x1080 | Higher |
| **2-lane** | 30fps | 1920x1200 | Lower |

**For Tank Project:**
- Recommended: **4-lane** (better performance)
- Resolution: 1280x720 @ 20fps (matches plan)

### **3. Source Files:**

**Download L4T 36.4.3 sources** (even for 6.2.1):
- `Jetson_Linux_R36.4.3_aarch64.tbz2`
- `Tegra_Linux_Sample-Root-Filesystem_R36.4.3_aarch64.tbz2`
- `public_sources.tbz2`

---

## 📝 Patch File Locations

**In your release package:**
```
~/Tank_projects/ecam-25docs/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_29-MAR-2025_R04/
└── e-CAM25_CUONX_L4T36.4.0_JP6.1.0_JETSON-ONX-ONANO_R04/
    └── ONX_ONANO/
        └── Kernel/
            ├── e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_module.patch
            ├── e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_dtb.patch
            └── e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_oot.patch
```

---

## ⚙️ Alternative: Use Pre-built Binaries (Not Recommended)

**Warning:** The release package has pre-built binaries for **JetPack 6.1** only. These may work on 6.2.1, but:

❌ Not officially supported  
❌ May have compatibility issues  
❌ May not work with all features

**If you want to try:**
```bash
cd ~/Tank_projects/ecam-25docs/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_29-MAR-2025_R04
sudo ./install_binaries.sh
```

**Better approach:** Build from source for 6.2.x as documented above.

---

## 📚 Reference Documents

**In your project:**
- `~/Tank_projects/ecam-25docs/econ25_upgradeto6.2.txt` - Official e-con upgrade guide
- `~/Tank_projects/ecam-25docs/BUILD_ECAM25_JP621_GUIDE.md` - Detailed build guide
- `~/Tank_projects/tank_ws/src/external/ECAM25_CAMERA_SETUP.md` - Setup documentation

**Official e-con Resources:**
- Developer Guide Manual (download from e-con Developer Resources)
- Release Notes (in your release package)

---

## ✅ Build Checklist

### **Before Building:**
- [ ] Linux PC ready (Ubuntu 20.04/22.04)
- [ ] At least 50GB free disk space
- [ ] NVIDIA Developer account (for downloads)
- [ ] e-CAM25 release package available
- [ ] Good internet connection

### **Download Checklist:**
- [ ] Bootlin toolchain downloaded
- [ ] L4T 36.4.3 driver package downloaded
- [ ] L4T 36.4.3 rootfs downloaded
- [ ] L4T 36.4.3 public sources downloaded
- [ ] e-CAM25 release package copied to build workspace

### **Build Checklist:**
- [ ] Environment variables set
- [ ] All sources extracted
- [ ] Module patch applied
- [ ] Device tree patch modified and applied
- [ ] nvidia-oot patch modified and applied
- [ ] vi5_fops.c fixed
- [ ] Kernel built successfully
- [ ] Modules built successfully
- [ ] DTBO built successfully

### **Installation Checklist:**
- [ ] DTBO copied to Jetson
- [ ] Kernel modules copied to Jetson
- [ ] ISP tuning file copied to Jetson
- [ ] Modules installed on Jetson
- [ ] DTBO configured in extlinux.conf
- [ ] System rebooted
- [ ] Camera verified (`/dev/video0` exists)

---

## 🚀 Quick Start Command Reference

**Full build process:** See `BUILD_ECAM25_JP621_GUIDE.md` for detailed step-by-step commands.

**Time estimate:** 2-3 hours (first time) including downloads and build.

---

**Summary:** Build drivers on Linux PC using L4T 36.4.3 sources (compatible with 6.2.1), apply patches with 6.2.x modifications, then install on Jetson.

