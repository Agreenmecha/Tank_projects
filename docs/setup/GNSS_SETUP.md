# ZED-F9P GNSS Setup Guide

## Overview

**Status:** ✅ Hardware connected, driver installed, ROS 2 node running

**Current Issue:** No position fix (expected indoors - needs clear sky view)

## Hardware Check

```bash
# Verify ZED-F9P is connected
ls -l /dev/serial/by-id/ | grep ublox
# Should show: usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00 -> ../../ttyACM0
```

## Quick Test

```bash
cd ~/Tank_projects
./test_gnss.sh
```

**Expected Results:**
- ✅ Node running: 31 UBX topics available
- ⚠️  No position data (indoors)
- ✅ When outside with clear sky: Position publishing at 5 Hz

---

## Configuration

### Automatic Configuration (ROS 2 Driver)

The `ublox_dgnss_node` automatically configures the ZED-F9P on startup using:

**File:** `tank_ws/src/tank_sensors/config/gnss_f9p.yaml`

Key settings:
- **Update rate:** 5 Hz (200ms between measurements)
- **Constellations:** GPS, GLONASS, Galileo, BeiDou, QZSS, SBAS
- **Dynamic model:** Automotive (ground vehicle)
- **Output:** UBX protocol (high-precision position data)

### Manual Configuration with u-center (Optional)

**When to use u-center:**
- Check satellite signals and fix quality
- View real-time sky plot
- Save configuration to flash memory
- Troubleshoot GNSS issues

#### Install u-center (Windows tool)

1. Download from: https://www.u-blox.com/en/product/u-center
2. Install on Windows laptop
3. Connect ZED-F9P via USB

#### Key u-center Features

**View → Messages View:**
- `UBX-NAV-PVT`: Position, velocity, time
- `UBX-NAV-HP-POS-LLH`: High-precision lat/lon
- `UBX-NAV-SAT`: Satellite signal strength

**View → Satellite Sky Plot:**
- See which satellites are visible
- Signal strength (SNR) for each
- Need 4+ satellites with good SNR for fix

**Configuration (CFG):**
The ROS 2 driver handles this automatically, but you can verify:
- `CFG-RATE`: 200ms measurement rate
- `CFG-GNSS`: All constellations enabled
- `CFG-NAV5`: Dynamic model = Automotive

**Save Configuration:**
```
Configuration View → CFG (Configuration) → Send → Save current configuration
```
This saves to flash, so settings persist after power cycle.

---

## Indoor vs Outdoor Performance

### Indoor (Current Status)
- ❌ No position fix expected
- GNSS signals are blocked by roof/walls
- Node is running and configured correctly
- Satellite acquisition impossible

### Outdoor (Expected Performance)
- ✅ Position fix within 30-60 seconds (cold start)
- ✅ 5-10 second fix (warm start, if recently used)
- ✅ Accuracy: 2-5m horizontal (without RTK)
- ✅ 5 Hz update rate
- ✅ 10-20 satellites tracked simultaneously

**Requirements for outdoor fix:**
- Clear view of sky (no buildings, trees overhead)
- Antenna placed horizontally on top of tank
- Wait 30-60 seconds for initial acquisition

---

## ROS 2 Topics

### Key Topics (when fix is acquired)

```bash
# High-precision position
ros2 topic echo /ubx_nav_hp_pos_llh

# Position/Velocity/Time
ros2 topic echo /ubx_nav_pvt

# Dilution of Precision (quality metric)
ros2 topic echo /ubx_nav_dop

# Satellite info
ros2 topic echo /ubx_nav_sat
```

### Sample Position Output (outdoor with fix)

```yaml
# /ubx_nav_hp_pos_llh
lon: -1215311143  # Longitude (deg * 1e-7)
lat: 394673255    # Latitude (deg * 1e-7)
height: 125430    # Height above ellipsoid (mm)
h_msl: 85220      # Height above mean sea level (mm)
lon_hp: -44       # High-precision longitude component
lat_hp: 82        # High-precision latitude component
height_hp: 5      # High-precision height component
h_msl_hp: -3
h_acc: 1250       # Horizontal accuracy estimate (mm) - e.g., 1.25m
v_acc: 2100       # Vertical accuracy estimate (mm) - e.g., 2.1m
```

**Conversion to decimal degrees:**
```
Latitude = (lat + lat_hp * 0.01) / 1e7
Longitude = (lon + lon_hp * 0.01) / 1e7
```

---

## Field Testing Procedure

### 1. Outdoor Setup
```bash
# SSH to Jetson (if using wireless AP)
ssh aaronjet@192.168.2.100

# Launch GNSS
cd ~/Tank_projects/tank_ws
source install/setup.bash
ros2 launch tank_sensors gnss.launch.py
```

### 2. Wait for Fix
- **Cold start:** 30-60 seconds (first time or after long power-off)
- **Warm start:** 5-15 seconds (recently used)
- Watch for messages like:
  ```
  [gnss_node]: Fix acquired - 12 satellites
  ```

### 3. Verify Position
```bash
# In new terminal
cd ~/Tank_projects
./test_gnss.sh

# Or manually check
ros2 topic echo /ubx_nav_hp_pos_llh --once
```

**Good fix indicators:**
- `h_acc` < 2500 (2.5m horizontal accuracy)
- `v_acc` < 5000 (5m vertical accuracy)
- 8+ satellites in `/ubx_nav_sat`
- Fix type: 3D fix (not 2D)

### 4. Check DOP (Quality Metric)
```bash
ros2 topic echo /ubx_nav_dop --once
```

**DOP values (lower is better):**
- `h_dop < 200` (2.0): Excellent
- `h_dop 200-300` (2.0-3.0): Good
- `h_dop 300-500` (3.0-5.0): Acceptable
- `h_dop > 500` (>5.0): Poor (wide corridor needed)

*DOP units are 0.01, so h_dop=150 means HDOP=1.5*

---

## Troubleshooting

### No Fix After 2 Minutes Outdoors

**Check antenna:**
```bash
# View satellite signals
ros2 topic echo /ubx_nav_sat
```
- Should show 10+ satellites
- SNR (signal-to-noise) > 20 for multiple satellites
- If no satellites, check antenna connection

**Check sky view:**
- Move to open area (away from buildings, trees)
- Place antenna horizontally on flat surface
- Ensure antenna ground plane is adequate

**Check configuration:**
```bash
# View node log
tail -100 /tmp/gnss.log
```

### Position Drifting

**Normal for standard GNSS (no RTK):**
- 2-5m accuracy is expected
- Position will "wander" slightly
- Use Point-LIO for local odometry, GNSS for global correction

**If drift is excessive (>10m):**
- Check DOP values (high DOP = poor geometry)
- Wait for more satellites to be acquired
- Check for interference (metal structures nearby)

### Topics Not Publishing

```bash
# Check node is running
ros2 node list | grep gnss

# Check for errors
tail -50 /tmp/gnss.log | grep ERROR

# Restart node
killall ublox_dgnss_node
ros2 launch tank_sensors gnss.launch.py
```

---

## Integration with Navigation

### Current Phase: Sensor Testing
- ✅ GNSS driver installed
- ✅ Node configured and running
- ⏳ Waiting for outdoor test (needs clear sky)

### Next Phase: EKF Fusion
Once GNSS is working outdoors, integrate with Point-LIO:

**File:** `tank_ws/src/tank_localization/config/ekf.yaml`

```yaml
ekf_filter_node:
  ros__parameters:
    # Fuse Point-LIO (local odometry) with GNSS (global position)
    odom0: /point_lio/odom
    navsat0: /ubx_nav_hp_pos_llh  # Use high-precision GNSS
    
    # Adapt corridor width based on GNSS quality
    # Read /ubx_nav_dop to adjust path following tolerance
```

### Corridor Width Adaptation
Based on GNSS DOP (from `/ubx_nav_dop`):

| HDOP      | Corridor Width | Use Case           |
|-----------|----------------|--------------------|
| < 2.0     | 4m             | Good fix, tight following |
| 2.0-3.0   | 5m             | Normal operations  |
| 3.0-5.0   | 6m             | Acceptable, wider margin |
| > 5.0     | 7-8m           | Poor fix, conservative |

---

## Summary

### What's Working Now ✅
- ZED-F9P connected and detected
- ublox_dgnss driver installed
- ROS 2 node running and configured
- 31 UBX topics available

### What Needs Testing 🔲
- **Outdoor position fix** (needs clear sky view)
- Verify 5 Hz update rate
- Verify 2-5m accuracy
- Long-term position stability

### Configuration Status
- **Automatic:** ROS 2 driver configures device via YAML ✅
- **Manual (u-center):** Optional, useful for verification
- **Flash memory:** Not saved yet (config resets on power cycle)
  - *Recommendation:* After verifying outdoor fix, save config in u-center

### Next Steps
1. **Take Jetson + ZED-F9P outside** with clear sky view
2. Run `./test_gnss.sh`
3. Verify position fix acquired (30-60 seconds)
4. Check accuracy: `h_acc` < 2500mm (2.5m)
5. (Optional) Use u-center to save config to flash

---

## Quick Reference

```bash
# Launch GNSS
ros2 launch tank_sensors gnss.launch.py

# Test status
~/Tank_projects/test_gnss.sh

# View position (outdoor only)
ros2 topic echo /ubx_nav_hp_pos_llh

# View satellite info
ros2 topic echo /ubx_nav_sat

# View fix quality (DOP)
ros2 topic echo /ubx_nav_dop

# Stop GNSS
killall ublox_dgnss_node
```

---

## Additional Resources

- **u-blox ZED-F9P Manual:** https://www.u-blox.com/en/product/zed-f9p-module
- **u-center Software:** https://www.u-blox.com/en/product/u-center
- **ROS 2 Driver:** https://github.com/aussierobots/ublox_dgnss
- **UBX Protocol:** https://www.u-blox.com/en/docs/UBX-18010854

---

**Status:** Ready for outdoor testing! 🌍

