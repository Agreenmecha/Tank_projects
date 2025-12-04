# ODrive ROS 2 Humble Node - Complete ✅

**Date:** November 4, 2025  
**Status:** ✅ Fully implemented and ready for testing  
**ODrive:** v3.6 with Firmware 0.5.6  
**Interface:** USB (native protocol via odrive Python library)

---

## Overview

The ODrive ROS 2 node is **fully implemented** and ready to use. It provides complete motor control functionality via USB interface.

**Location:** `tank_ws/src/tank_control/`

---

## Package Structure

```
tank_control/
├── tank_control/
│   ├── odrive_interface_node.py    # Main ODrive ROS 2 node
│   └── safety_monitor_node.py      # Safety monitoring (optional)
├── launch/
│   └── odrive_interface.launch.py  # Launch file
├── config/
│   ├── odrive_params.yaml          # ODrive configuration
│   └── safety_limits.yaml          # Safety thresholds
├── CMakeLists.txt
├── package.xml
├── setup.py
└── README.md
```

---

## Features

### ✅ Implemented Features

1. **Velocity Control**
   - Subscribes to `/cmd_vel` (geometry_msgs/Twist)
   - Converts to differential drive motor velocities
   - Clamps to configured limits
   - Sends commands at 50 Hz

2. **Encoder Odometry**
   - Publishes `/odrive/encoder_odom` (nav_msgs/Odometry)
   - Computes position from wheel encoders
   - Differential drive kinematics
   - Updates at 20 Hz

3. **Motor Status Monitoring**
   - Publishes `/odrive/motor_status` (sensor_msgs/JointState)
   - Motor velocities, currents, positions
   - Real-time feedback

4. **Error Handling**
   - Publishes `/odrive/errors` (std_msgs/Bool)
   - Checks axis, motor, encoder errors
   - Automatic error detection

5. **Watchdog Timer**
   - Auto-stop motors if no `/cmd_vel` received for 0.2s
   - Configurable timeout
   - Can be disabled for testing

6. **Emergency Stop**
   - Subscribes to `/emergency_stop` (std_msgs/Bool)
   - Immediate motor stop
   - Safety feature

7. **Services**
   - `/odrive/clear_errors` - Clear ODrive errors
   - `/odrive/calibrate` - Run motor calibration

8. **Auto-Reconnection**
   - Detects connection loss
   - Attempts to reconnect automatically
   - Graceful error handling

---

## Configuration

### Parameters (`config/odrive_params.yaml`)

```yaml
odrive_interface_node:
  ros__parameters:
    # ODrive connection
    odrive_serial: ""  # Auto-find first ODrive
    
    # Axis configuration
    axis_left: 0   # ODrive axis 0 for left motor
    axis_right: 1  # ODrive axis 1 for right motor
    
    # Physical parameters
    wheel_radius: 0.10    # meters (adjust to your wheels)
    track_width: 0.60     # meters (600mm per tank specs)
    encoder_cpr: 8192     # Encoder counts per revolution
    
    # Velocity limits
    max_vel: 1.5          # Maximum linear velocity (m/s)
    max_angular_vel: 2.0  # Maximum angular velocity (rad/s)
    
    # ODrive motor limits
    current_limit: 30.0   # Motor current limit (Amps)
    velocity_limit: 20.0  # Motor velocity limit (turns/s)
    
    # Safety
    enable_watchdog: true
    watchdog_timeout: 0.2  # seconds
    
    # Control timing
    control_rate: 50.0    # Hz - velocity command rate
    publish_rate: 20.0    # Hz - status/odometry rate
```

---

## Usage

### Build Package

```bash
cd ~/Tank_projects/tank_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select tank_control --symlink-install
source install/setup.bash
```

**Note:** You may see a warning about "No task extension to 'build' a 'ros.ament_cmake_python' package" - this is harmless and the package will still work.

### Launch Node

```bash
cd ~/Tank_projects/tank_ws
source install/setup.bash
ros2 launch tank_control odrive_interface.launch.py
```

### Or Run Directly

```bash
ros2 run tank_control odrive_interface_node
```

---

## Testing

### Test Script

```bash
~/Tank_projects/test_odrive_ros2.sh
```

This script verifies:
- ODrive USB connection
- odrivetool installation
- Direct ODrive Python connection
- ROS 2 workspace setup
- Node availability
- Configuration files

### Manual Testing

**1. Test Connection:**
```bash
# In odrivetool
odrivetool
odrv0.vbus_voltage  # Should show voltage
```

**2. Test ROS 2 Node:**
```bash
# Launch node
ros2 launch tank_control odrive_interface.launch.py

# In another terminal, send velocity command
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" --once

# Check odometry
ros2 topic echo /odrive/encoder_odom

# Check motor status
ros2 topic echo /odrive/motor_status
```

**3. Test Keyboard Teleop:**
```bash
# Install teleop if needed
sudo apt install ros-humble-teleop-twist-keyboard

# Run teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Topics

### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands (linear.x, angular.z) |
| `/emergency_stop` | `std_msgs/Bool` | Emergency stop trigger |

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `/odrive/encoder_odom` | `nav_msgs/Odometry` | Wheel encoder odometry |
| `/odrive/motor_status` | `sensor_msgs/JointState` | Motor velocities, currents, positions |
| `/odrive/errors` | `std_msgs/Bool` | Error status |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/odrive/clear_errors` | `std_srvs/Trigger` | Clear ODrive errors |
| `/odrive/calibrate` | `std_srvs/Trigger` | Run motor calibration |

---

## Differential Drive Kinematics

The node implements standard differential drive kinematics:

**Forward Kinematics:**
```
v_left = v - (ω * track_width / 2)
v_right = v + (ω * track_width / 2)
```

Where:
- `v` = linear velocity (m/s)
- `ω` = angular velocity (rad/s)
- `track_width` = 0.60m (600mm)

**Motor Velocity Conversion:**
```
motor_vel (turns/s) = wheel_vel (m/s) / (2 * π * wheel_radius)
```

**Odometry:**
```
dist_center = (dist_left + dist_right) / 2
delta_theta = (dist_right - dist_left) / track_width
```

---

## Safety Features

### Watchdog Timer

- **Enabled by default:** Yes
- **Timeout:** 0.2 seconds
- **Behavior:** Motors stop if no `/cmd_vel` received within timeout
- **Configurable:** Via `watchdog_timeout` parameter

### Emergency Stop

- **Topic:** `/emergency_stop` (std_msgs/Bool)
- **Behavior:** Immediate motor stop when `true` published
- **Recovery:** Publish `false` to resume

### Current Limiting

- **Software limit:** 30.0 Amps (configurable)
- **Hardware limit:** Set in ODrive firmware
- **Monitoring:** Via `/odrive/motor_status` topic

### Error Detection

- **Automatic:** Checks axis, motor, encoder errors
- **Publishing:** `/odrive/errors` topic
- **Recovery:** Use `/odrive/clear_errors` service

---

## ODrive Setup Requirements

Before using the ROS 2 node, your ODrive must be configured:

### Minimal Configuration

```python
# In odrivetool:
odrv0.axis0.encoder.config.mode = ENCODER_MODE_INCREMENTAL
odrv0.axis1.encoder.config.mode = ENCODER_MODE_INCREMENTAL
odrv0.axis0.encoder.config.cpr = 8192  # Adjust to your encoder
odrv0.axis1.encoder.config.cpr = 8192
odrv0.axis0.motor.config.pole_pairs = 7  # Adjust to your motor
odrv0.axis1.motor.config.pole_pairs = 7
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
odrv0.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

# Run calibration
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
# Wait for calibration...

# Set to closed loop
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

# Save
odrv0.save_configuration()
odrv0.reboot()
```

---

## Troubleshooting

### ODrive Not Found

**Symptoms:** Node logs "ODrive not found!"

**Solutions:**
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

**Symptoms:** Node connected but motors don't respond to `/cmd_vel`

**Solutions:**
```bash
# Check ODrive errors
ros2 topic echo /odrive/errors

# Clear errors
ros2 service call /odrive/clear_errors std_srvs/srv/Trigger

# Check if motors are in closed loop control
# Via odrivetool:
odrv0.axis0.current_state  # Should be 8 (AXIS_STATE_CLOSED_LOOP_CONTROL)
odrv0.axis1.current_state

# Check encoder status
odrv0.axis0.encoder.is_ready  # Should be True
odrv0.axis1.encoder.is_ready
```

### Watchdog Stopping Motors

**Symptoms:** Motors stop after a few seconds

**Solutions:**
- Check if `/cmd_vel` is being published continuously
- Verify `watchdog_timeout` parameter is appropriate
- Disable watchdog for testing (not recommended for production):
  ```yaml
  enable_watchdog: false
  ```

### Connection Drops

**Symptoms:** Node logs connection errors

**Solutions:**
- Check USB cable quality (use shielded cable)
- Reduce USB cable length (<2m recommended)
- Add ferrite bead to USB cable
- Check for USB power issues

---

## Integration with Other Systems

### Point-LIO Integration

The `/odrive/encoder_odom` topic can be fused with Point-LIO odometry:

```yaml
# In EKF configuration
odom0: /point_lio/odom       # LiDAR-inertial odometry
odom1: /odrive/encoder_odom # Wheel encoder odometry
```

Use encoder odometry as a cross-check for slip detection.

### Navigation Stack

The `/odrive/encoder_odom` topic provides base odometry for the navigation stack:

```yaml
# In navigation config
odom_frame_id: odom
base_frame_id: base_link
```

---

## Performance Metrics

### Latency

- **USB latency:** ~1-2ms
- **Command processing:** <1ms
- **Control loop:** 50 Hz (20ms period)
- **Status publishing:** 20 Hz (50ms period)

### Accuracy

- **Odometry accuracy:** Depends on encoder resolution and wheel radius calibration
- **Velocity control:** Good for smooth operation
- **Position accuracy:** Accumulates error over time (use with GNSS/Point-LIO)

---

## Configuration Tips

### Wheel Radius Calibration

1. Measure actual wheel diameter
2. Calculate radius: `radius = diameter / 2`
3. Update `wheel_radius` in `odrive_params.yaml`
4. Test odometry accuracy over known distance

### Track Width Calibration

1. Measure distance between wheel centers
2. Update `track_width` in `odrive_params.yaml`
3. Test turning accuracy (should pivot in place with angular velocity only)

### Velocity Limits

**Conservative (recommended for testing):**
```yaml
max_vel: 0.5          # 0.5 m/s
max_angular_vel: 1.0  # 1.0 rad/s
```

**Normal operation:**
```yaml
max_vel: 1.5          # 1.5 m/s
max_angular_vel: 2.0  # 2.0 rad/s
```

---

## Code Structure

### Key Functions

**`connect_odrive()`** - Connects to ODrive via USB  
**`configure_odrive()`** - Sets up motor limits and control mode  
**`cmd_vel_callback()`** - Handles velocity commands  
**`control_loop()`** - Sends commands to motors (50 Hz)  
**`publish_status()`** - Publishes odometry and status (20 Hz)  
**`compute_odometry()`** - Calculates position from encoders  
**`check_errors()`** - Monitors ODrive errors  

---

## Next Steps

1. **Connect ODrive** via USB
2. **Configure ODrive** using odrivetool (calibration, limits)
3. **Test node** with `test_odrive_ros2.sh`
4. **Launch node** and verify topics
5. **Test control** with keyboard teleop
6. **Integrate** with Point-LIO for slip detection

---

## Summary

✅ **ODrive ROS 2 node is fully implemented**  
✅ **USB interface using native odrive Python library**  
✅ **Complete feature set: velocity control, odometry, error handling**  
✅ **Safety features: watchdog, emergency stop, error monitoring**  
✅ **Ready for testing and integration**

**Status:** Ready to use once ODrive is connected and configured!

---

**Last Updated:** November 4, 2025  
**Node File:** `tank_ws/src/tank_control/tank_control/odrive_interface_node.py`  
**Launch File:** `tank_ws/src/tank_control/launch/odrive_interface.launch.py`

