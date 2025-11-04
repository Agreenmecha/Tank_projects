# ODrive Quickstart for Jetson Orin Nano

**Hardware:** ODrive v3.6  
**Firmware:** v0.5.6  
**ODrive Tool:** v0.5.4 (tested and working)  
**Platform:** Jetson Orin Nano (Ubuntu 20.04/22.04)  
**Date:** November 3, 2025

---

## ⚠️ Important Version Compatibility

**Use ODrive control utility v0.5.4 with Firmware v0.5.6**

- ✅ **ODrive tool v0.5.4** works with FW v0.5.6
- ❌ **Newer versions** (v0.5.5+, v0.6.x) may NOT work with older FW v0.5.6
- ❌ **Older versions** (v0.5.3-) may have compatibility issues

**DO NOT use `pip install odrive` without specifying version!**

---

## 📋 Prerequisites

- Jetson Orin Nano with JetPack 6.x
- ODrive v3.6 with Firmware v0.5.6
- USB cable (Type-A to Micro-B)
- Two brushless motors with encoders
- Appropriate power supply for motors (12-48V depending on setup)

---

## 🚀 Installation Steps

### 1. Install ODrive Tool (Specific Version)

```bash
# Install Python 3 and pip if not already installed
sudo apt update
sudo apt install python3 python3-pip -y

# Install ODrive tool v0.5.4 (IMPORTANT: specify version!)
pip3 install odrive==0.5.4

# Verify installation
odrivetool --version
# Should show: ODrive control utility v0.5.4
```

### 2. Fix USB Permissions

```bash
# Add user to dialout group for USB access
sudo usermod -a -G dialout $USER

# Create udev rule for ODrive
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0d[0-9][0-9]", MODE="0666"' | sudo tee /etc/udev/rules.d/50-odrive.rules

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Log out and back in for group changes to take effect
# OR reboot
sudo reboot
```

### 3. Connect ODrive

```bash
# After reboot, connect ODrive via USB

# Verify USB connection
lsusb | grep ODrive
# Should show: Bus XXX Device XXX: ID 1209:0d32 ODrive Robotics ODrive v3.x

# Check USB device appears
ls -la /dev/ttyACM*
# Should show: /dev/ttyACM0 or similar
```

### 4. Test Connection with odrivetool

```bash
# Launch odrivetool
odrivetool

# In odrivetool, it should find your ODrive automatically
# You'll see something like:
# Connected to ODrive 206A396A3548 as odrv0
```

**If connection fails:**
```bash
# Try specifying the device
odrivetool --path /dev/ttyACM0
```

---

## ⚙️ First-Time ODrive Configuration

Run these commands in `odrivetool` to configure your ODrive for the first time:

### Step 1: Check Firmware Version

```python
# In odrivetool:
odrv0.fw_version_major, odrv0.fw_version_minor, odrv0.fw_version_revision
# Should show: (0, 5, 6) or similar
```

### Step 2: Configure Encoders

```python
# Set encoder mode (incremental encoders with AB signals)
odrv0.axis0.encoder.config.mode = ENCODER_MODE_INCREMENTAL
odrv0.axis1.encoder.config.mode = ENCODER_MODE_INCREMENTAL

# Set encoder counts per revolution (CPR)
# IMPORTANT: Adjust this to your encoder specification!
odrv0.axis0.encoder.config.cpr = 8192
odrv0.axis1.encoder.config.cpr = 8192

# If you have an index signal (Z channel), enable it:
# odrv0.axis0.encoder.config.use_index = True
# odrv0.axis1.encoder.config.use_index = True
```

### Step 3: Configure Motors

```python
# Set motor type to high current (for most BLDC motors)
odrv0.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
odrv0.axis1.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT

# Set pole pairs (check your motor datasheet)
# Common values: 7, 14, 21
odrv0.axis0.motor.config.pole_pairs = 7
odrv0.axis1.motor.config.pole_pairs = 7

# Set current limits (Amps)
# Start conservative, increase later if needed
odrv0.axis0.motor.config.current_lim = 30.0
odrv0.axis1.motor.config.current_lim = 30.0

# Set calibration current (Amps) - for motor calibration
odrv0.axis0.motor.config.calibration_current = 10.0
odrv0.axis1.motor.config.calibration_current = 10.0
```

### Step 4: Configure Control Mode

```python
# Set control mode to velocity control
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
odrv0.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

# Set input mode to passthrough (direct velocity commands)
odrv0.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
odrv0.axis1.controller.config.input_mode = INPUT_MODE_PASSTHROUGH

# Set velocity limit (turns/s)
odrv0.axis0.controller.config.vel_limit = 20.0
odrv0.axis1.controller.config.vel_limit = 20.0
```

### Step 5: Save Configuration

```python
# Save all settings to non-volatile memory
odrv0.save_configuration()

# ODrive will reboot automatically
# Wait ~5 seconds, then reconnect:
# exit() and restart odrivetool
```

---

## 🧪 Motor Calibration

**⚠️ IMPORTANT:** Motors must be able to spin freely during calibration!

### Step 1: Run Full Calibration Sequence

```python
# In odrivetool (after reconnecting):

# Axis 0 calibration
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

# Wait for motor to spin and calibrate (~10-15 seconds)
# Motor will beep and rotate

# Check for errors
odrv0.axis0.error
# Should be 0

odrv0.axis0.motor.error
odrv0.axis0.encoder.error
# Both should be 0

# Repeat for Axis 1
odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
# Wait...
odrv0.axis1.error
# Should be 0
```

### Step 2: Set Pre-Calibrated (Optional but Recommended)

```python
# If calibration was successful, mark as pre-calibrated
# This skips calibration on future boots
odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis1.motor.config.pre_calibrated = True

odrv0.axis0.encoder.config.pre_calibrated = True
odrv0.axis1.encoder.config.pre_calibrated = True

# Save configuration
odrv0.save_configuration()
```

---

## 🎮 Basic Motor Control Test

### Test 1: Enter Closed Loop Control

```python
# In odrivetool:

# Start closed loop control on both axes
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

# Check state (should be 8 = AXIS_STATE_CLOSED_LOOP_CONTROL)
odrv0.axis0.current_state
# Should show: 8
```

### Test 2: Velocity Control

```python
# Command velocity (turns/s)
# Start slow!
odrv0.axis0.controller.input_vel = 0.5
odrv0.axis1.controller.input_vel = 0.5

# Motors should spin slowly

# Check actual velocity
odrv0.axis0.encoder.vel_estimate
odrv0.axis1.encoder.vel_estimate

# Stop motors
odrv0.axis0.controller.input_vel = 0
odrv0.axis1.controller.input_vel = 0
```

### Test 3: Check Encoder Position

```python
# Read encoder positions
odrv0.axis0.encoder.pos_estimate
odrv0.axis1.encoder.pos_estimate

# Manually spin motor, position should change
```

---

## 🔧 Common Issues & Solutions

### Issue: "ODrive not found"

**Solution:**
```bash
# Check USB connection
lsusb | grep ODrive

# Check permissions
groups | grep dialout
# If dialout not shown, log out and back in

# Try specifying device manually
odrivetool --path /dev/ttyACM0
```

### Issue: "ModuleNotFoundError: No module named 'odrive'"

**Solution:**
```bash
# Make sure you installed with pip3, not pip
pip3 install odrive==0.5.4

# Check installation
pip3 list | grep odrive
# Should show: odrive 0.5.4
```

### Issue: Motor calibration fails (error != 0)

**Solution:**
```python
# Clear errors first
odrv0.clear_errors()

# Check what the error was
hex(odrv0.axis0.error)
hex(odrv0.axis0.motor.error)
hex(odrv0.axis0.encoder.error)

# Common causes:
# - Motor not connected properly
# - Encoder not connected properly
# - Current limit too low for motor
# - Motor can't spin freely (mechanical obstruction)

# Increase calibration current if needed
odrv0.axis0.motor.config.calibration_current = 15.0
odrv0.save_configuration()
```

### Issue: Motor shakes/vibrates in closed loop

**Solution:**
```python
# Reduce velocity gain (makes control less aggressive)
odrv0.axis0.controller.config.vel_gain *= 0.5
odrv0.axis1.controller.config.vel_gain *= 0.5

# Increase velocity integrator gain (improves steady-state)
odrv0.axis0.controller.config.vel_integrator_gain *= 1.5
odrv0.axis1.controller.config.vel_integrator_gain *= 1.5

# Save
odrv0.save_configuration()
```

### Issue: "ImportError: libusb-1.0.so.0: cannot open shared object"

**Solution:**
```bash
sudo apt install libusb-1.0-0
```

---

## 🐍 Python Script Test

Save this as `test_odrive_basic.py`:

```python
#!/usr/bin/env python3
import odrive
import time

# Find ODrive
print("Searching for ODrive...")
odrv = odrive.find_any(timeout=10)

if odrv is None:
    print("ERROR: ODrive not found!")
    exit(1)

print(f"Connected to ODrive: {odrv.serial_number}")
print(f"Firmware: v{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")
print(f"Bus voltage: {odrv.vbus_voltage:.2f}V")

# Enter closed loop control
print("\nEntering closed loop control...")
odrv.axis0.requested_state = 8  # AXIS_STATE_CLOSED_LOOP_CONTROL
odrv.axis1.requested_state = 8

time.sleep(1)

# Test velocity control
print("Testing velocity control (0.5 turns/s for 2 seconds)...")
odrv.axis0.controller.input_vel = 0.5
odrv.axis1.controller.input_vel = 0.5

time.sleep(2)

print("Stopping...")
odrv.axis0.controller.input_vel = 0.0
odrv.axis1.controller.input_vel = 0.0

print("\nDone!")
```

Run it:
```bash
python3 test_odrive_basic.py
```

---

## 🤖 ROS2 Integration

Once ODrive is working with odrivetool, proceed to ROS2 integration:

```bash
# Navigate to workspace
cd ~/Tank_projects/tank_ws

# Build the tank_control package
colcon build --packages-select tank_control
source install/setup.bash

# Run the test script first
cd ~/Tank_projects
python3 test_odrive_usb.py

# If test passes, launch ROS2 motor control
ros2 launch tank_control motor_control.launch.py
```

---

## 📊 Typical Configuration Summary

For reference, here's a typical working configuration:

```python
# Encoders
odrv0.axis0.encoder.config.mode = ENCODER_MODE_INCREMENTAL
odrv0.axis0.encoder.config.cpr = 8192

# Motors  
odrv0.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
odrv0.axis0.motor.config.pole_pairs = 7
odrv0.axis0.motor.config.current_lim = 30.0
odrv0.axis0.motor.config.calibration_current = 10.0

# Controller
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
odrv0.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
odrv0.axis0.controller.config.vel_limit = 20.0

# Pre-calibrated (after successful calibration)
odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis0.encoder.config.pre_calibrated = True
```

---

## 📝 Quick Command Reference

```python
# Connection
odrv0 = odrive.find_any()

# Calibration
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

# Control
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.requested_state = AXIS_STATE_IDLE

# Velocity
odrv0.axis0.controller.input_vel = 1.0

# Monitoring
odrv0.vbus_voltage
odrv0.axis0.encoder.pos_estimate
odrv0.axis0.encoder.vel_estimate
odrv0.axis0.motor.current_control.Iq_measured
odrv0.axis0.error

# Save/Reboot
odrv0.save_configuration()
odrv0.reboot()
odrv0.clear_errors()
```

---

## 🎯 Next Steps

After successful ODrive setup:

1. ✅ Verify both motors work in odrivetool
2. ✅ Test with Python script
3. ✅ Measure actual wheel radius for tank
4. ✅ Update `config/odrive_params.yaml` with correct values
5. ✅ Launch ROS2 motor control stack
6. ✅ Test with keyboard teleop
7. ✅ Integrate with Point-LIO localization

---

## 📚 Reference

- **ODrive Documentation:** https://docs.odriverobotics.com/v/0.5.6/
- **ODrive Forum:** https://discourse.odriverobotics.com/
- **Tank Control Package:** `~/Tank_projects/tank_ws/src/tank_control/README.md`
- **Test Script:** `~/Tank_projects/test_odrive_usb.py`

---

**Version:** 1.0  
**Last Updated:** November 3, 2025  
**Status:** Ready for Jetson deployment

---

## 💡 Tips

- Always start with **low current limits** (10-15A) and increase gradually
- Test with **one axis at a time** initially
- Keep motors **mechanically free** during calibration
- Monitor **vbus_voltage** to ensure adequate power supply
- Use **pre_calibrated = True** to skip calibration on boot (after successful calibration)
- **Log errors** with `hex(odrv0.axis0.error)` for troubleshooting

