# Tank Control Package

Motor control interface for ODrive v3.6 with Firmware 0.5.6 over USB.

## Overview

This package provides:
- **ODrive USB Interface** - Velocity control for differential drive
- **Encoder Odometry** - Wheel-based odometry from motor encoders
- **Safety Monitor** - Temperature, current, pitch angle monitoring
- **Watchdog** - Automatic motor stop on command timeout

## Hardware Requirements

- **ODrive v3.6** with Firmware 0.5.6
- **USB connection** to Jetson Orin Nano
- **2 motors** with encoders (left on axis 0, right on axis 1)

## Installation

### 1. Install ODrive Python Library

**IMPORTANT:** Use ODrive tool v0.5.4 with Firmware v0.5.6!

```bash
# Install specific version (v0.5.4 tested with FW 0.5.6)
pip3 install odrive==0.5.4

# Verify version
odrivetool --version
# Should show: ODrive control utility v0.5.4
```

**⚠️ Warning:** Newer versions (v0.5.5+, v0.6.x) may NOT work with FW v0.5.6!

### 2. Build Package

```bash
cd ~/Tank_projects/tank_ws
colcon build --packages-select tank_control
source install/setup.bash
```

## ODrive Setup (First Time)

Before using with ROS2, configure your ODrive via `odrivetool`:

```bash
odrivetool
```

### Minimal Configuration

```python
# In odrivetool:

# Set encoder mode (incremental)
odrv0.axis0.encoder.config.mode = ENCODER_MODE_INCREMENTAL
odrv0.axis1.encoder.config.mode = ENCODER_MODE_INCREMENTAL

# Set encoder CPR (counts per revolution)
odrv0.axis0.encoder.config.cpr = 2048  # Verified with hardware
odrv0.axis1.encoder.config.cpr = 2048

# Set motor pole pairs (adjust for your motor)
odrv0.axis0.motor.config.pole_pairs = 7  # Adjust for your motor
odrv0.axis1.motor.config.pole_pairs = 7

# Set current limits (Amps)
odrv0.axis0.motor.config.current_lim = 30
odrv0.axis1.motor.config.current_lim = 30

# Set velocity limits (turns/s)
odrv0.axis0.controller.config.vel_limit = 20
odrv0.axis1.controller.config.vel_limit = 20

# Set control mode to velocity control
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
odrv0.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

# Run motor calibration
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
# Wait for calibration to complete...

# Set to closed loop control
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

# Save configuration
odrv0.save_configuration()
odrv0.reboot()
```

## Configuration

Edit `config/odrive_params.yaml` to match your robot:

```yaml
wheel_radius: 0.10    # meters (measure your wheel diameter)
track_width: 0.60     # meters (distance between wheels)
encoder_cpr: 2048     # encoder counts per revolution (verified with hardware)
max_vel: 1.5          # maximum linear velocity (m/s)
current_limit: 30.0   # motor current limit (Amps)
```

## Usage

### Launch ODrive Interface Only

```bash
ros2 launch tank_control odrive_interface.launch.py
```

### Launch Full Motor Control Stack (with safety)

```bash
ros2 launch tank_control motor_control.launch.py
```

### Send Velocity Commands

```bash
# Move forward at 0.5 m/s
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# Turn in place (pivot right)
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"

# Stop
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

### Keyboard Teleoperation

```bash
# Install teleop_twist_keyboard if needed
sudo apt install ros-humble-teleop-twist-keyboard

# Run teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Topics

### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/emergency_stop` | `std_msgs/Bool` | Emergency stop trigger |

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `/odrive/encoder_odom` | `nav_msgs/Odometry` | Wheel encoder odometry |
| `/odrive/motor_status` | `sensor_msgs/JointState` | Motor velocities, currents, positions |
| `/odrive/errors` | `std_msgs/Bool` | ODrive error status |
| `/safety/status` | `std_msgs/String` | Safety status messages |
| `/safety/warnings` | `std_msgs/String` | Warning messages |

## Services

| Service | Type | Description |
|---------|------|-------------|
| `/odrive/clear_errors` | `std_srvs/Trigger` | Clear ODrive errors |
| `/odrive/calibrate` | `std_srvs/Trigger` | Run motor calibration |

## Monitoring

### Check Motor Status

```bash
ros2 topic echo /odrive/motor_status
```

### Check Odometry

```bash
ros2 topic echo /odrive/encoder_odom
```

### Check for Errors

```bash
ros2 topic echo /odrive/errors
```

### Clear Errors

```bash
ros2 service call /odrive/clear_errors std_srvs/srv/Trigger
```

## Safety Features

### Watchdog Timer
- Motors automatically stop if no `/cmd_vel` received for 0.2 seconds
- Configurable via `watchdog_timeout` parameter
- Can be disabled for testing (not recommended)

### Current Limiting
- Software current limits configured in `odrive_params.yaml`
- Hardware limits configured in ODrive firmware
- Monitor via `/odrive/motor_status` topic

### Pitch Angle Monitoring
- Monitors IMU pitch angle (requires `/imu/data` topic)
- Warning at 28°, critical at 32°, emergency stop at 38°
- Configurable in `safety_limits.yaml`

### Emergency Stop
- Publish `true` to `/emergency_stop` to stop motors
- Automatically triggered by safety monitor on critical conditions
- Motors remain stopped until `false` is published

## Troubleshooting

### ODrive Not Found

```bash
# Check USB connection
lsusb | grep ODrive

# Check permissions
sudo usermod -a -G dialout $USER
# Log out and back in

# Test with odrivetool
odrivetool
```

### Motors Not Moving

```bash
# Check ODrive errors
ros2 topic echo /odrive/errors

# Clear errors
ros2 service call /odrive/clear_errors std_srvs/srv/Trigger

# Check if motors are in closed loop control
# Via odrivetool:
dev0.axis0.current_state  # Should be AXIS_STATE_CLOSED_LOOP_CONTROL (8)
dev0.axis1.current_state
```

### Encoder Not Working

```bash
# Check encoder configuration in odrivetool
odrv0.axis0.encoder.config.cpr  # Should match your encoder
odrv0.axis0.encoder.is_ready    # Should be True

# Check encoder count is changing
odrv0.axis0.encoder.pos_estimate  # Manually spin motor, this should change
```

### Connection Drops

- Check USB cable quality
- Use shielded USB cable for noisy environments
- Add ferrite bead to USB cable
- Reduce USB cable length (<2m)

## Parameters Reference

### odrive_params.yaml

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `odrive_serial` | string | "" | ODrive serial number (empty = auto-find) |
| `axis_left` | int | 0 | ODrive axis for left motor |
| `axis_right` | int | 1 | ODrive axis for right motor |
| `wheel_radius` | float | 0.10 | Wheel radius in meters |
| `track_width` | float | 0.60 | Distance between wheels in meters |
| `encoder_cpr` | int | 2048 | Encoder counts per revolution (verified with hardware) |
| `max_vel` | float | 1.5 | Maximum linear velocity (m/s) |
| `max_angular_vel` | float | 2.0 | Maximum angular velocity (rad/s) |
| `current_limit` | float | 30.0 | Motor current limit (Amps) |
| `velocity_limit` | float | 20.0 | Motor velocity limit (turns/s) |
| `watchdog_timeout` | float | 0.2 | Watchdog timeout (seconds) |
| `enable_watchdog` | bool | true | Enable watchdog timer |
| `control_rate` | float | 50.0 | Control loop rate (Hz) |
| `publish_rate` | float | 20.0 | Status publish rate (Hz) |

## Notes

- **Firmware 0.5.6** is tested and working
- USB interface has ~1-2ms latency
- For production use on rough terrain, consider CAN bus for better noise immunity
- Always test with low velocities first (0.1-0.2 m/s)
- Calibrate motors before first use

## Future Improvements

- [ ] CAN bus interface option
- [ ] Motor temperature monitoring (requires firmware mod or CAN)
- [ ] GPIO E-stop integration
- [ ] Track slip detection (compare encoder odom vs Point-LIO)
- [ ] Automatic motor calibration on startup
- [ ] Better error recovery

---

**Status:** Ready for testing on Jetson with ODrive connected via USB

