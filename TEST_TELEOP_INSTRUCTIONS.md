# ROS2 Gamepad Teleop Test Instructions

## What We Created

**ROS2 Gamepad Teleop Node** that publishes to `/cmd_vel` to test the entire ODrive control stack:

```
Gamepad → teleop_gamepad_node → /cmd_vel → odrive_interface_node → ODrive Motors
```

## Files Created

1. **`tank_ws/src/tank_control/tank_control/teleop_gamepad_node.py`**
   - ROS2 node that reads gamepad input
   - Publishes `Twist` messages to `/cmd_vel`
   - Emergency stop on X button
   - Quit on Square button

2. **`tank_ws/src/tank_control/launch/teleop_gamepad.launch.py`**
   - Launch file with configurable parameters
   - Max velocities, deadzone, publish rate

3. **Updated `setup.py` and `CMakeLists.txt`**
   - Added teleop_gamepad_node to console_scripts
   - Installed launch files

## How to Test on Rover

### Step 1: Start ODrive Interface Node

```bash
cd ~/Tank_projects/tank_ws
source install/setup.bash
export ROS_DOMAIN_ID=42

# Launch ODrive interface
ros2 launch tank_control odrive_interface.launch.py
```

This will:
- Connect to ODrive
- Subscribe to `/cmd_vel`
- Publish to `/odrive/encoder_odom`
- Handle emergency stops

### Step 2: Start Teleop Node (in new terminal/tmux)

```bash
cd ~/Tank_projects/tank_ws
source install/setup.bash
export ROS_DOMAIN_ID=42

# Launch gamepad teleop
ros2 launch tank_control teleop_gamepad.launch.py
```

Or run the node directly:

```bash
ros2 run tank_control teleop_gamepad_node
```

### Step 3: Control the Tank!

**Controls:**
- **Left Stick Y**: Forward/Backward
- **Right Stick X**: Turn Left/Right
- **X Button**: Emergency Stop (hold to stop, release to resume)
- **Square Button**: Quit

## What This Tests

✅ **ROS2 Control Pipeline:**
- Gamepad input → ROS node
- `/cmd_vel` topic communication
- ODrive interface subscription
- Velocity command conversion

✅ **Gearbox Ratio & Wheel Radius:**
- Motor velocity = wheel_velocity × 12.0
- Correct wheel radius (58mm)
- Track width (600mm)

✅ **Odometry:**
- Encoder readings → wheel turns ÷ 12.0
- Distance calculation
- Published to `/odrive/encoder_odom`

✅ **Safety:**
- Emergency stop handling
- Watchdog timeout
- Clean shutdown

## Monitoring

### Watch cmd_vel commands:
```bash
ros2 topic echo /cmd_vel
```

### Watch odometry:
```bash
ros2 topic echo /odrive/encoder_odom
```

### Check ODrive status:
```bash
ros2 topic echo /odrive/status
```

### Monitor topics:
```bash
ros2 topic list
ros2 topic hz /cmd_vel
ros2 topic hz /odrive/encoder_odom
```

## Troubleshooting

### Gamepad not detected:
```bash
ls /dev/input/js*
# Should see /dev/input/js0

# Test with jstest
sudo apt install joystick
jstest /dev/input/js0
```

### pygame not installed:
```bash
pip3 install pygame
```

### ODrive not connecting:
```bash
# Check USB connection
lsusb | grep ODrive

# Test with odrivetool
odrivetool
```

### No movement:
1. Check ODrive is in CLOSED_LOOP_CONTROL mode
2. Verify axes are enabled
3. Check emergency stop is not active
4. Monitor `/cmd_vel` - are commands being published?
5. Check ODrive status topic for errors

## Next Steps After Testing

Once this works, we have a fully functional ROS2 control stack ready for:

1. **GPS Waypoint Navigation**
   - Add GPS localization node
   - Implement waypoint follower
   - Integrate with existing `/cmd_vel` interface

2. **LiDAR Obstacle Avoidance**
   - Add local costmap
   - Implement collision avoidance
   - Override `/cmd_vel` when obstacles detected

3. **Autonomous Navigation**
   - Global path planner (GPS waypoints)
   - Local path planner (LiDAR obstacles)
   - Behavior tree for mission control

---

**Status**: Code synced to rover, ready to test! 🎮🤖

