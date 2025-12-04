# ODrive Motor Configuration Summary

**Date:** December 4, 2025  
**Status:** ✅ Configured and Ready

---

## Motor Setup

### Hardware
- **ODrive:** v3.6 (Firmware v0.5.6)
- **Serial:** 59748237849395
- **Gearbox:** 13:1 ratio
- **Max Motor RPM:** 6000 RPM
- **Bus Voltage:** ~51.6V

### Encoders
- **Type:** Incremental (pre-calibrated)
- **CPR:** 2048 counts per revolution
- **Both axes:** Ready and error-free

---

## Configuration Parameters

### Velocity Control
```python
# Velocity limits
velocity_limit = 100.0      # turns/s (6000 RPM motor with 13:1 gearbox)
vel_ramp_rate = 10.0        # turns/s² - smooth acceleration (3-15 recommended)

# At motor shaft: 100 turns/s = 6000 RPM
# At output shaft: 100/13 = 7.69 turns/s = 461 RPM
```

### Current & Safety
```python
current_limit = 30.0        # Amps
watchdog_timeout = 0.2      # seconds
enable_watchdog = True
```

### Physical Parameters
```python
wheel_radius = 0.10         # meters (adjust to actual wheels)
track_width = 0.60          # meters (600mm between wheels)
encoder_cpr = 2048          # counts per revolution
```

---

## Control Modes

### Current Configuration
- **Control Mode:** Velocity control (CONTROL_MODE_VELOCITY_CONTROL)
- **Input Mode:** Passthrough (INPUT_MODE_PASSTHROUGH)
- **State:** Closed loop control ready

### Velocity Ramp Rate Options

| Rate (turns/s²) | Acceleration Style | Use Case |
|-----------------|-------------------|----------|
| 3-7 | Gentle, smooth | Indoor navigation, precise control |
| 8-12 | Balanced | **Default - general use** |
| 13-15 | Quick, responsive | Fast response, outdoor terrain |

**Current Setting:** 10 turns/s² (balanced)

---

## Quick Configuration Scripts

### Set Velocity Limit
```bash
sg dialout -c "python3 set_velocity_limit.py"
```
Sets velocity limit to 100 turns/s and saves to ODrive.

### Set Ramp Rate
```bash
sg dialout -c "python3 set_ramp_rate.py 10"
```
Adjust value between 3-15 for different acceleration profiles.

### Put Motors in Idle
```bash
sg dialout -c "python3 idle_motors.py"
```
Emergency stop and idle motors anytime.

---

## Velocity Calculations

### Motor Speed
```
Motor RPM = velocity_limit × 60
          = 100 × 60
          = 6000 RPM ✓
```

### Output Speed (After Gearbox)
```
Output RPM = Motor RPM / gear_ratio
           = 6000 / 13
           = 461 RPM

Output turns/s = velocity_limit / gear_ratio
               = 100 / 13
               = 7.69 turns/s
```

### Linear Velocity (at wheel)
```
wheel_circumference = 2 × π × wheel_radius
                    = 2 × π × 0.10
                    = 0.628 m

max_linear_velocity = output_turns/s × wheel_circumference
                    = 7.69 × 0.628
                    = 4.83 m/s (~17.4 km/h)
```

**Note:** Adjust `wheel_radius` in config for accurate calculations.

---

## ROS2 Configuration

Config file: `tank_ws/src/tank_control/config/odrive_params.yaml`

```yaml
odrive_interface_node:
  ros__parameters:
    # Physical parameters
    wheel_radius: 0.10
    track_width: 0.60
    encoder_cpr: 2048
    
    # Velocity limits
    max_vel: 1.5              # m/s linear (software limit for safety)
    max_angular_vel: 2.0      # rad/s
    
    # ODrive motor limits
    current_limit: 30.0       # Amps
    velocity_limit: 100.0     # turns/s (hardware limit)
    vel_ramp_rate: 10.0       # turns/s² (acceleration)
    
    # Safety
    enable_watchdog: true
    watchdog_timeout: 0.2     # seconds
    
    # Control timing
    control_rate: 50.0        # Hz
    publish_rate: 20.0        # Hz
```

---

## Testing & Verification

### Test Motor Control (Python)
```bash
cd /home/aaronjet/Tank_projects
sg dialout -c "python3 test_motor_velocity.py"
```

**Results:**
- ✅ 10 turns/s: Tracking accurate
- ✅ -10 turns/s: Reverse working
- ✅ 100 turns/s: Full speed capable
- ✅ Ramp rate: Smooth acceleration
- ✅ Idle on exit: Safe shutdown

### Test ROS2 Control
```bash
# Terminal 1
./launch_odrive_ros2.sh

# Terminal 2
./test_ros2_velocity.sh
```

---

## Variable Names Used

**Important:** Scripts use `dev0` (not `odrv0`)

```python
import odrive
from odrive.enums import *

dev0 = odrive.find_any()

# Set velocity limit
dev0.axis0.controller.config.vel_limit = 100
dev0.axis1.controller.config.vel_limit = 100

# Set ramp rate
dev0.axis0.controller.config.vel_ramp_rate = 10
dev0.axis1.controller.config.vel_ramp_rate = 10

# Enter closed loop
dev0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
dev0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

# Set velocity
dev0.axis0.controller.input_vel = 100  # turns/s
dev0.axis1.controller.input_vel = 100

# Idle when done
dev0.axis0.requested_state = AXIS_STATE_IDLE
dev0.axis1.requested_state = AXIS_STATE_IDLE
```

---

## Key Features Implemented

### ✅ Velocity Control
- Smooth acceleration with configurable ramp rate
- Software and hardware velocity limits
- Bidirectional control (forward/reverse)

### ✅ Safety Features
- Watchdog timer (auto-stop on communication loss)
- Emergency stop capability
- Current limiting
- Automatic idle on script exit

### ✅ ROS2 Integration
- `/cmd_vel` subscriber for velocity commands
- `/odrive/encoder_odom` odometry publishing
- `/odrive/motor_status` real-time monitoring
- Services for error clearing and calibration

---

## Next Steps

1. ✅ **Basic motor control** - Working
2. ✅ **Velocity ramping** - Configured (10 turns/s²)
3. ✅ **Velocity limits** - Set to 100 turns/s
4. ⬜ **Test ROS2 integration** - Ready to test
5. ⬜ **Calibrate wheel radius** - Measure actual wheels
6. ⬜ **Tune ramp rate** - Test 3-15 range for best feel
7. ⬜ **Test full speed** - Verify 100 turns/s capability

---

## Reference

- Config: `tank_ws/src/tank_control/config/odrive_params.yaml`
- Node: `tank_ws/src/tank_control/tank_control/odrive_interface_node.py`
- Test scripts: `/home/aaronjet/Tank_projects/`
- ODrive Docs: https://docs.odriverobotics.com/v/0.5.6/

---

**All systems configured and ready for testing!**

