# NVIDIA Downloads Required for e-CAM25 Camera Build

**Date:** November 4, 2025  
**For:** JetPack 6.2.1 (L4T 36.4.7) on Jetson Orin Nano  
**Build Platform:** Linux PC (Ubuntu 20.04/22.04)

---

## 🎯 Quick Summary

**You need to download 4 files from NVIDIA:**

1. ✅ **Bootlin Toolchain** (from Bootlin, not NVIDIA)
2. ✅ **NVIDIA Driver Package (BSP)** - L4T 36.4.3
3. ✅ **NVIDIA Sample Root Filesystem** - L4T 36.4.3
4. ✅ **NVIDIA Public Sources** - L4T 36.4.3

**Total size:** ~6.5GB  
**Note:** Use L4T 36.4.3 sources (compatible with 6.2.1)

---

## 📥 Download Instructions

### Step 1: NVIDIA Developer Account

1. Go to: **https://developer.nvidia.com/**
2. Create account or log in
3. You'll need this for downloading NVIDIA files

### Step 2: Navigate to JetPack 6.2

1. Go to: **https://developer.nvidia.com/embedded/jetpack**
2. Find **JetPack 6.2** (NOT 6.2.1)
3. Click "Download" or "Get Started"

### Step 3: Download Files

Look for **L4T 36.4.3** downloads (these work for 6.2.1)

---

## 📦 File 1: Bootlin Cross-Compilation Toolchain

**File Name:** `aarch64--glibc--stable-2022.08-1.tar.bz2`  
**Size:** ~100MB  
**Source:** Bootlin (not NVIDIA)

### Direct Download:
```
https://toolchains.bootlin.com/downloads/releases/toolchains/aarch64/tarballs/aarch64--glibc--stable-2022.08-1.tar.bz2
```

### Or Browse:
1. Go to: **https://toolchains.bootlin.com/**
2. Navigate to: **aarch64** → **glibc** → **stable-2022.08-1**
3. Download: `aarch64--glibc--stable-2022.08-1.tar.bz2`

**No login required!**

---

## 📦 File 2: NVIDIA Driver Package (BSP)

**File Name:** `Jetson_Linux_R36.4.3_aarch64.tbz2`  
**Size:** ~1.5GB  
**Source:** NVIDIA Developer Portal

### Where to Download:

1. **Go to:** https://developer.nvidia.com/embedded/jetpack
2. **Log in** with your NVIDIA Developer account
3. **Find:** JetPack 6.2 → L4T 36.4.3
4. **Look for:** "Driver Package (BSP)" or "Jetson Linux"
5. **Download:** `Jetson_Linux_R36.4.3_aarch64.tbz2`

### Direct Link (requires login):
```
https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v4.3/release/jetson_linux_r36.4.3_aarch64.tbz2
```

### Alternative Names You Might See:
- `Jetson_Linux_R36.4.3_aarch64.tbz2`
- `jetson_linux_r36.4.3_aarch64.tbz2`
- "Driver Package (BSP)" for JetPack 6.2

---

## 📦 File 3: NVIDIA Sample Root Filesystem

**File Name:** `Tegra_Linux_Sample-Root-Filesystem_R36.4.3_aarch64.tbz2`  
**Size:** ~3GB  
**Source:** NVIDIA Developer Portal

### Where to Download:

1. **Go to:** https://developer.nvidia.com/embedded/jetpack
2. **Log in** with your NVIDIA Developer account
3. **Find:** JetPack 6.2 → L4T 36.4.3
4. **Look for:** "Sample Root Filesystem" or "Rootfs"
5. **Download:** `Tegra_Linux_Sample-Root-Filesystem_R36.4.3_aarch64.tbz2`

### Direct Link (requires login):
```
https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v4.3/release/tegra_linux_sample-root-filesystem_r36.4.3_aarch64.tbz2
```

### Alternative Names You Might See:
- `Tegra_Linux_Sample-Root-Filesystem_R36.4.3_aarch64.tbz2`
- `tegra_linux_sample-root-filesystem_r36.4.3_aarch64.tbz2`
- "Sample Root Filesystem" for JetPack 6.2

---

## 📦 File 4: NVIDIA Public Sources

**File Name:** `public_sources.tbz2`  
**Size:** ~2GB  
**Source:** NVIDIA Developer Portal

### Where to Download:

1. **Go to:** https://developer.nvidia.com/embedded/jetpack
2. **Log in** with your NVIDIA Developer account
3. **Find:** JetPack 6.2 → L4T 36.4.3
4. **Look for:** "Public Sources" or "Kernel Sources"
5. **Download:** `public_sources.tbz2`

### Direct Link (requires login):
```
https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v4.3/sources/public_sources.tbz2
```

### Alternative Names You Might See:
- `public_sources.tbz2`
- "Public Sources" for JetPack 6.2
- "Kernel Sources"

---

## 📋 Complete Download Checklist

Print this checklist and check off each file:

- [ ] **Bootlin Toolchain**
  - [ ] File: `aarch64--glibc--stable-2022.08-1.tar.bz2`
  - [ ] Size: ~100MB
  - [ ] Source: https://toolchains.bootlin.com/

- [ ] **NVIDIA Driver Package (BSP)**
  - [ ] File: `Jetson_Linux_R36.4.3_aarch64.tbz2`
  - [ ] Size: ~1.5GB
  - [ ] Source: NVIDIA Developer Portal (requires login)

- [ ] **NVIDIA Sample Root Filesystem**
  - [ ] File: `Tegra_Linux_Sample-Root-Filesystem_R36.4.3_aarch64.tbz2`
  - [ ] Size: ~3GB
  - [ ] Source: NVIDIA Developer Portal (requires login)

- [ ] **NVIDIA Public Sources**
  - [ ] File: `public_sources.tbz2`
  - [ ] Size: ~2GB
  - [ ] Source: NVIDIA Developer Portal (requires login)

**Total:** ~6.5GB

---

## 📍 Where to Place Files

**After downloading, place all files in:**
```
~/jetson_camera_build/
```

**Or whatever directory you plan to use for the build.**

---

## ⚠️ Important Notes

### Why L4T 36.4.3 for JetPack 6.2.1?

- JetPack 6.2.1 (L4T 36.4.7) uses the **same kernel** as JetPack 6.2 (L4T 36.4.3)
- Both use Linux Kernel 5.15
- Binaries built from 6.2 sources work on 6.2.1
- **Do NOT download L4T 36.4.7 sources** - they don't exist as separate packages!

### File Verification

After downloading, verify file integrity:

```bash
# Check file sizes (approximate)
ls -lh *.tbz2 *.tar.bz2

# Expected sizes:
# aarch64--glibc--stable-2022.08-1.tar.bz2      ~100MB
# jetson_linux_r36.4.3_aarch64.tbz2            ~1.5GB
# tegra_linux_sample-root-filesystem_r36.4.3_aarch64.tbz2  ~3GB
# public_sources.tbz2                          ~2GB
```

### If Download Links Don't Work

1. **Check NVIDIA Developer Portal:**
   - Go to: https://developer.nvidia.com/embedded/jetpack
   - Navigate to JetPack 6.2
   - Look for "Download" or "Get Started" buttons

2. **Try NVIDIA Developer Forums:**
   - Links may be updated
   - Community often shares direct links

3. **Contact NVIDIA Support:**
   - If you have an NVIDIA Developer account
   - They can provide direct download links

---

## 🚀 Quick Download Command (if you have wget/curl)

**Note:** These commands require you to be logged into NVIDIA Developer Portal in your browser first, or you'll need to provide authentication.

```bash
# Create download directory
mkdir -p ~/jetson_camera_build
cd ~/jetson_camera_build

# Download Bootlin toolchain (no login needed)
wget https://toolchains.bootlin.com/downloads/releases/toolchains/aarch64/tarballs/aarch64--glibc--stable-2022.08-1.tar.bz2

# For NVIDIA files, you'll need to:
# 1. Log in to developer.nvidia.com in your browser
# 2. Navigate to JetPack 6.2 download page
# 3. Right-click download links and copy
# 4. Use wget with cookies or use browser download
```

**Better approach:** Use browser download with NVIDIA Developer Portal login.

---

## ✅ Verification After Download

Once all files are downloaded, verify:

```bash
cd ~/jetson_camera_build

# List all files
ls -lh

# Should show:
# aarch64--glibc--stable-2022.08-1.tar.bz2
# jetson_linux_r36.4.3_aarch64.tbz2
# tegra_linux_sample-root-filesystem_r36.4.3_aarch64.tbz2
# public_sources.tbz2

# Check total size
du -sh .
# Should be approximately 6.5GB
```

---

## 📞 Need Help?

**If you can't find the files:**
1. Check NVIDIA Developer Portal: https://developer.nvidia.com/embedded/jetpack
2. Look for "JetPack 6.2" (not 6.2.1)
3. Check for "L4T 36.4.3" downloads
4. Contact NVIDIA support if you have developer account

**If downloads are slow:**
- Use a download manager
- Download during off-peak hours
- Consider using a different network connection

---

**Ready to build once all files are downloaded!** 🚀

Run: `./build_ecam25_drivers.sh` from the `ecam-25docs` directory.

