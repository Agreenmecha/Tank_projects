# CAN Bus Hardware Setup Guide

**Hardware:** Adafruit CAN Pal (TJA1051T/3) + Jetson Orin Nano  
**Configuration:** Native CAN FD controller (mttcan)  
**Date:** 2025-10-31

---

## ✅ Hardware Confirmed

### **Adafruit CAN Pal - Product 5708**
- **Part:** TJA1051T/3 CAN FD Transceiver
- **Link:** https://www.adafruit.com/product/5708
- **Price:** $3.95
- **Type:** Transceiver only (NOT SPI controller)

**Key Features:**
- ✅ CAN FD capable (up to 5 Mbps)
- ✅ 3.3V logic compatible
- ✅ Built-in 5V charge pump (no external 5V needed!)
- ✅ Switchable 120Ω termination resistor
- ✅ Pre-soldered terminal block

---

## 🔌 Wiring Diagram

### **CAN Pal to Jetson Orin Nano J17 Connector**

```
Adafruit CAN Pal          Jetson Orin Nano J17
────────────────          ───────────────────
VCC (3.3V logic)    →     Pin 17 (3.3V PWR)
GND                 →     Pin 20 (GND)
RX (to transceiver) →     Pin 1 (CAN_RX)
TX (from transceiver) →   Pin 2 (CAN_TX)
```

### **CAN Pal Terminal Block to ODrive**

```
CAN Pal Terminal Block    ODrive CAN Port
──────────────────────    ───────────────
CANH                 →    CANH
CANL                 →    CANL
GND (optional)       →    GND (improves noise immunity)
```

---

## ⚙️ Hardware Setup Steps

### 1. Prepare CAN Pal
- [ ] Solder header pins to VCC, GND, RX, TX pads
- [ ] Flip termination switch to **ON** (enables 120Ω resistor)
- [ ] No need to solder terminal block pins (pre-soldered)

### 2. Wire to Jetson
- [ ] Connect VCC → J17 Pin 17 (3.3V)
- [ ] Connect GND → J17 Pin 20 (GND)
- [ ] Connect RX → J17 Pin 1 (CAN_RX)
- [ ] Connect TX → J17 Pin 2 (CAN_TX)

### 3. Connect ODrive
- [ ] Wire CANH from terminal block → ODrive CANH
- [ ] Wire CANL from terminal block → ODrive CANL
- [ ] (Optional) Wire GND → ODrive GND for better noise immunity
- [ ] Enable ODrive's 120Ω termination (check your ODrive model docs)

### 4. Configure ODrive (via USB first)
```bash
# Connect ODrive via USB, then run:
odrivetool

# In odrivetool:
odrv0.config.enable_can_a = True
odrv0.axis0.config.can_node_id = 0  # Left motor
odrv0.axis1.config.can_node_id = 1  # Right motor
odrv0.can.config.baud_rate = 500000
odrv0.save_configuration()
odrv0.reboot()
```

---

## 🧪 Testing

### On Jetson (after hardware setup):

```bash
# Run the test script
cd ~/Tank_projects
sudo ./test_can_setup.sh

# Expected results:
# ✓ CAN modules loaded (can, can_raw, mttcan)
# ✓ can0 interface found
# ✓ CAN interface state: ERROR-ACTIVE
# ✓ CAN loopback working
# ✓ ODrive heartbeat detected
```

### Manual testing:
```bash
# Load modules
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan

# Configure CAN interface (500 kbps, CAN FD with 2 Mbps data rate)
sudo ip link set can0 type can bitrate 500000 dbitrate 2000000 fd on
sudo ip link set up can0

# Check status
ip -details link show can0
# Should show: "state ERROR-ACTIVE" (good!)

# Listen for ODrive heartbeat
candump can0
# Should see periodic messages like:
#   can0  001   [8]  00 00 00 00 01 00 00 00
```

---

## 🐛 Troubleshooting

### Problem: "BUS-OFF" state

**Causes:**
- Termination switch not flipped ON
- ODrive termination not enabled
- Wiring issue (CANH/CANL swapped)
- No device on bus

**Fix:**
1. Check CAN Pal termination switch is **ON**
2. Check ODrive termination is enabled
3. Verify CANH/CANL wiring
4. Check power to CAN Pal (3.3V)

### Problem: No ODrive heartbeat detected

**Causes:**
- ODrive CAN not enabled
- Wrong baud rate
- Incorrect node ID

**Fix:**
1. Connect ODrive via USB and run odrivetool
2. Verify: `odrv0.config.enable_can_a == True`
3. Verify: `odrv0.can.config.baud_rate == 500000`
4. Check: `odrv0.axis0.config.can_node_id` is set (0 or 1)

### Problem: "can0" interface not found

**Causes:**
- CAN not enabled in device tree
- mttcan module not loaded

**Fix:**
```bash
# Check device tree
cat /proc/device-tree/mttcan@c310000/status
# Should show: "okay"

# If shows "disabled", CAN is not enabled (contact NVIDIA support or check Jetson-IO)
```

---

## 📊 CAN FD Capabilities

**Jetson Orin Nano CAN Controller (mttcan):**
- Classic CAN: 10 kbps to 1 Mbps
- CAN FD arbitration phase: up to 1 Mbps
- CAN FD data phase: **up to 15 Mbps** (5 Mbps typical with most transceivers)

**Adafruit CAN Pal (TJA1051T/3):**
- CAN FD data rate: **up to 5 Mbps** (certified)

**Practical Setup:**
- Arbitration bitrate: 500 kbps (standard for ODrive)
- Data bitrate: 2 Mbps (CAN FD, for future expansion)

```bash
# Configure with CAN FD:
sudo ip link set can0 type can bitrate 500000 dbitrate 2000000 fd on
```

---

## 🔮 Future Expansion

With CAN FD enabled, you can easily add:
- **Teensy 4.1** with CAN FD support
- Additional ODrive controllers
- CAN-based sensors (IMUs, encoders, etc.)
- Other robots/devices on the same bus

All will be able to communicate at high speed on the same 2-wire bus!

---

## 📚 References

**Official Docs:**
- [NVIDIA Jetson CAN Controller](https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/HR/ControllerAreaNetworkCan.html)
- [Adafruit CAN Pal Product Page](https://www.adafruit.com/product/5708)
- [Adafruit CAN Pal Learn Guide](https://learn.adafruit.com/adafruit-can-pal)

**Project Docs:**
- `workspace_structure.md` - Detailed CAN setup section
- `test_can_setup.sh` - Automated test script
- `test_odrive_can.py` - Python CAN test script

---

**Status:** Ready for Phase 1 implementation! ✅

