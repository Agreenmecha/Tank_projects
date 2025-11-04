# u-center 2 Configuration Guide for ZED-F9P

## Objective
Configure ZED-F9P to output UBX messages (not NMEA) for use with ROS 2.

---

## Prerequisites
- ✅ ZED-F9P connected to Windows laptop via USB
- ✅ u-center 2 installed and running
- ✅ Device connected and receiving data (24/42 satellites - excellent!)

---

## Step-by-Step Configuration

### 1. Open Configuration View
- In u-center 2, go to: **View → Configuration View**
- You should see a tree of configuration items on the left

---

### 2. Disable NMEA Output on USB

**Find:** `CFG-USB` or search for "USB protocol"

Look for these settings and configure:

```
CFG-USBOUTPROT-NMEA = 0    ❌ Disable NMEA on USB
CFG-USBOUTPROT-UBX = 1     ✅ Enable UBX on USB
CFG-USBOUTPROT-RTCM3X = 0  ❌ Disable RTCM (not needed now)
```

**Alternative location:** Under `PRT (Ports)` → `USB` → Output Protocol
- ✅ Check: **UBX**
- ❌ Uncheck: **NMEA**

---

### 3. Disable INFO Messages (Less Clutter)

**Find:** `CFG-INFMSG` (Information Messages)

```
CFG-INFMSG-UBX_USB = 0     ❌ Disable info messages on USB
CFG-INFMSG-NMEA_USB = 0    ❌ Disable NMEA info messages
```

---

### 4. Enable Required UBX Messages

**Find:** `CFG-MSGOUT` (Message Output Configuration)

This section has MANY items. Search for each of these and set them:

#### **USB Output Messages (set to 1 = output every measurement)**

```
CFG-MSGOUT-UBX_NAV_PVT_USB = 1          ✅ Position/Velocity/Time
CFG-MSGOUT-UBX_NAV_HPPOSLLH_USB = 1     ✅ High-Precision Lat/Lon/Height
CFG-MSGOUT-UBX_NAV_COV_USB = 1          ✅ Covariance (for EKF fusion)
CFG-MSGOUT-UBX_NAV_DOP_USB = 1          ✅ Dilution of Precision
CFG-MSGOUT-UBX_NAV_STATUS_USB = 1       ✅ Fix status
CFG-MSGOUT-UBX_NAV_SAT_USB = 1          ✅ Satellite info
```

**Optional (useful but not critical):**
```
CFG-MSGOUT-UBX_NAV_SIG_USB = 1          Signal quality per satellite
CFG-MSGOUT-UBX_NAV_VELNED_USB = 1       Velocity in NED frame
```

**Tip:** Use the search box in u-center 2 to quickly find these items.

---

### 5. Set Update Rate

**Find:** `CFG-RATE` (Rate Settings)

```
CFG-RATE-MEAS = 200        200 milliseconds = 5 Hz update rate
CFG-RATE-NAV = 1           Navigation solution every measurement
CFG-RATE-TIMEREF = 0       UTC time reference
```

**Why 5 Hz?** Good balance between update frequency and CPU load for ground vehicle navigation.

---

### 6. Verify GNSS Constellations (Should Already Be Good)

**Find:** `CFG-SIGNAL` (GNSS Signal Configuration)

These should already be enabled (you have 24/42 satellites):

```
CFG-SIGNAL-GPS_ENA = 1        ✅ GPS
CFG-SIGNAL-GAL_ENA = 1        ✅ Galileo
CFG-SIGNAL-GLO_ENA = 1        ✅ GLONASS
CFG-SIGNAL-BDS_ENA = 1        ✅ BeiDou
CFG-SIGNAL-QZSS_ENA = 1       ✅ QZSS
CFG-SIGNAL-SBAS_ENA = 1       ✅ SBAS (WAAS/EGNOS)
```

---

### 7. Set Dynamic Model (Optional but Recommended)

**Find:** `CFG-NAVSPG` (Navigation Signal Processing)

```
CFG-NAVSPG-DYNMODEL = 4       4 = Automotive (for ground vehicles)
CFG-NAVSPG-FIXMODE = 0        0 = Auto (2D/3D)
```

---

### 8. **IMPORTANT: Send Configuration to Device**

After changing each setting, you need to send it:

**Method A (Per-setting):**
- After changing a value, click **"Send"** button next to that setting

**Method B (All at once):**
- After changing all values, go to: **Configuration → CFG (Configuration)**
- Click **"Send"** to apply all changes

---

### 9. **CRITICAL: Save to Flash Memory**

This makes your configuration permanent (persists after power cycle):

**Option A: Configuration View**
1. Go to: **Configuration → CFG (Configuration)**
2. Find **"Save current configuration"** button
3. Click it

**Option B: Menu**
1. Go to: **Receiver → Action**
2. Select **"Save Configuration"**
3. Choose: **All sections**
4. Click **"OK"**

**You should see:** A confirmation message that configuration was saved.

---

### 10. Verify Configuration

Before disconnecting, verify you're getting UBX messages:

**Check Messages View:**
- Go to: **View → Messages View**
- You should now see **UBX messages** like:
  - `UBX-NAV-PVT` (updating at 5 Hz)
  - `UBX-NAV-HPPOSLLH` (high-precision position)
  - `UBX-NAV-DOP` (quality metrics)
- You should see **fewer or no NMEA messages** ($GNGGA, $GNRMC, etc.)

---

## Quick Configuration Checklist

Before saving, verify:

- [ ] `CFG-USBOUTPROT-UBX = 1` ✅
- [ ] `CFG-USBOUTPROT-NMEA = 0` ❌
- [ ] `CFG-MSGOUT-UBX_NAV_PVT_USB = 1` ✅
- [ ] `CFG-MSGOUT-UBX_NAV_HPPOSLLH_USB = 1` ✅
- [ ] `CFG-MSGOUT-UBX_NAV_COV_USB = 1` ✅
- [ ] `CFG-MSGOUT-UBX_NAV_DOP_USB = 1` ✅
- [ ] `CFG-MSGOUT-UBX_NAV_STATUS_USB = 1` ✅
- [ ] `CFG-MSGOUT-UBX_NAV_SAT_USB = 1` ✅
- [ ] `CFG-RATE-MEAS = 200` (5 Hz) ✅
- [ ] **Saved to flash memory** ✅

---

## After Configuration

1. **Disconnect from u-center 2**
2. **Unplug ZED-F9P from laptop**
3. **Plug into Jetson via USB**
4. **Test with ROS 2:**

```bash
# SSH to Jetson
ssh aaronjet@192.168.1.XX

# Stop old node (if running)
killall ublox_dgnss_node

# Start GNSS node
cd ~/Tank_projects/tank_ws
source install/setup.bash
ros2 launch tank_sensors gnss.launch.py
```

5. **In another terminal, test:**

```bash
cd ~/Tank_projects
./test_gnss.sh
```

---

## Expected Results After Configuration

```
1. Checking GNSS node...
✓ GNSS node is running

2. Available GNSS topics:
   ... (31 total topics)

3. Testing key topics...
   a) NAV-PVT (Position/Velocity/Time):
      ✓ Publishing at ~5 Hz

   b) NAV-HP-POS-LLH (High-Precision Lat/Lon):
      ✓ Publishing at ~5 Hz

   c) NAV-DOP (Dilution of Precision):
      ✓ Publishing at ~5 Hz

4. Sample position data:
✓ Position received!
  lat: 394673706
  lon: -1215311034
  h_acc: 1250  (1.25m accuracy)
  v_acc: 2100  (2.1m accuracy)
```

---

## Troubleshooting

### Still seeing NMEA messages in ROS 2 log?
- Configuration not saved to flash
- Go back to u-center and **save configuration** again
- Power cycle the ZED-F9P after saving

### No UBX messages in u-center after configuration?
- Make sure you clicked **"Send"** after changing values
- Try: **Receiver → Reset** → Choose "Software Reset"
- Reconfigure and save again

### Can't find specific CFG items?
- Use the **search box** in Configuration View
- Different u-center versions may organize items differently
- Look under both `CFG-*` and legacy `PRT/MSG` sections

---

## Summary

**Time required:** 5-10 minutes

**What you're doing:**
- Switching from NMEA protocol (text-based, low precision) to UBX protocol (binary, high precision)
- Enabling specific messages needed for ROS 2 navigation
- Setting 5 Hz update rate
- Saving everything to flash memory

**Result:**
- ZED-F9P will output high-precision UBX messages
- ROS 2 will receive position data at 5 Hz
- Configuration persists across power cycles

---

**Ready to configure? Let me know when you're done, and we'll test on the Jetson!** 🛰️

