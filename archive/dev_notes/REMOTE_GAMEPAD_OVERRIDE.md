# Remote Gamepad Override for Autonomous Navigation

Allows a remote gamepad to override Nav2's autonomous commands at any time for safety.

---

## Architecture

```
┌──────────────┐
│   Gamepad    │ (priority: 100)
│  (Laptop)    │
└──────┬───────┘
       │ /cmd_vel_joy
       │
       ▼
┌─────────────────────────┐
│     Twist Mux           │ ◄── Prioritizes inputs
│  (on Jetson)            │
└──────┬──────────────────┘
       │ /cmd_vel
       │
       ▼
┌─────────────────────────┐
│   ODrive Interface      │
│   (Motor Controller)    │
└─────────────────────────┘

       ▲
       │ /cmd_vel_nav
       │
┌──────┴───────┐
│    Nav2      │ (priority: 10)
│  Autonomous  │
└──────────────┘
```

**Priority System:**
- **200**: Emergency safety stop (highest)
- **100**: Manual gamepad control
- **10**: Autonomous Nav2 (lowest)

**Timeout:** If no gamepad input for 500ms, autonomous takes over.

---

## Quick Setup

### 1. Install Dependencies (already done)

```bash
sudo apt install ros-humble-twist-mux ros-humble-topic-tools
```

### 2. Launch Full System

```bash
cd ~/Tank_projects/tank_ws
source install/setup.bash
ros2 launch tank_bringup full_autonomy.launch.py
```

### 3. Launch Gamepad on Laptop

**Terminal 1 (launch gamepad driver):**
```bash
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
ros2 launch teleop_twist_joy teleop-launch.py joy_config:=xbox
```

**Terminal 2 (remap to correct topic for twist_mux):**
```bash
ros2 run topic_tools relay /cmd_vel /cmd_vel_joy
```

Or create a combined launch file (see below).

---

## Laptop Gamepad Launch File

Create on your **laptop** (not Jetson):

**`~/remote_gamepad.launch.py`:**

```python
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Joy node (reads gamepad)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',  # Or js1, js2, etc.
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }]
    )
    
    # Teleop twist joy (converts joy to cmd_vel)
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[{
            'axis_linear.x': 1,   # Left stick up/down
            'axis_angular.yaw': 0,  # Left stick left/right
            'scale_linear.x': 1.0,
            'scale_angular.yaw': 2.0,
            'enable_button': 4,   # LB button (hold to enable)
        }],
        remappings=[
            ('/cmd_vel', '/cmd_vel_joy'),  # Output to twist_mux input
        ]
    )
    
    return LaunchDescription([
        joy_node,
        teleop_node,
    ])
```

**Usage:**
```bash
export ROS_DOMAIN_ID=42
ros2 launch remote_gamepad.launch.py
```

---

## Testing

### 1. Verify Twist Mux is Running

```bash
ros2 node list | grep twist_mux
# Should show: /twist_mux
```

### 2. Check Topics

```bash
ros2 topic list | grep cmd_vel
```

**Expected:**
- `/cmd_vel` (output to ODrive)
- `/cmd_vel_joy` (from gamepad)
- `/cmd_vel_nav` (from Nav2)
- `/cmd_vel_safety` (for emergency stop)

### 3. Monitor Active Input

```bash
ros2 topic echo /twist_mux/selected
```

**Shows which input is active:**
- `gamepad` when gamepad publishing
- `navigation` when autonomous
- `safety` if emergency stop active

### 4. Test Override

**Start autonomous navigation:**
```bash
# In RViz, set a 2D Nav Goal
```

**While robot is moving autonomously:**
- Press **LB button** on gamepad
- Move left stick → robot should respond to gamepad
- Release LB button → after 500ms timeout, Nav2 resumes control

---

## Configuration

Edit `tank_ws/src/tank_control/config/twist_mux.yaml`:

```yaml
/**:
  ros__parameters:
    topics:
      gamepad:
        topic: /cmd_vel_joy
        timeout: 0.5  # How long to wait after last gamepad input
        priority: 100
        
      navigation:
        topic: /cmd_vel_nav
        timeout: 1.0
        priority: 10
      
      safety:
        topic: /cmd_vel_safety
        timeout: 0.5
        priority: 200  # Highest priority
    
    cmd_vel_out: /cmd_vel
```

**Tuning timeout:**
- **Lower (0.2s)**: Faster switch to autonomous, but gamepad must update faster
- **Higher (1.0s)**: Slower switch, but more forgiving of gamepad lag

---

## Gamepad Button Mappings

### Xbox Controller

```
          [LB]──────Hold to enable teleop
           │
           │
    [Left Stick]
        ↑↓   Linear velocity (forward/back)
        ←→   Angular velocity (turn)
    
    [RB]  Emergency stop (optional)
```

### PlayStation Controller

```
          [L1]──────Hold to enable teleop
           │
           │
    [Left Stick]
        ↑↓   Linear velocity (forward/back)
        ←→   Angular velocity (turn)
    
    [R1]  Emergency stop (optional)
```

---

## Safety Features

### 1. Dead Man's Switch
- **LB/L1 button must be held** to send gamepad commands
- If released, twist_mux times out → Nav2 takes over

### 2. Priority Override
- Gamepad (100) always overrides Nav2 (10)
- Safety (200) overrides everything

### 3. Network Loss Protection
- If gamepad disconnects, timeout triggers
- Nav2 continues autonomous operation
- **Warning:** Robot will NOT stop automatically

---

## Emergency Stop

### Option 1: Gamepad Button

Add to your gamepad launch:

```python
'enable_turbo_button': 5,  # RB button
'scale_linear_turbo.x': 0.0,  # Turbo = stop
'scale_angular_turbo.yaw': 0.0,
```

Press **RB** to send zero velocity.

### Option 2: Emergency Stop Topic

From laptop:

```bash
ros2 topic pub /cmd_vel_safety geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --rate 10
```

This has highest priority (200).

---

## Troubleshooting

### Gamepad Not Overriding Nav2

**Check gamepad is publishing:**
```bash
ros2 topic hz /cmd_vel_joy
```

**Check twist_mux selection:**
```bash
ros2 topic echo /twist_mux/selected
```

**Verify priority in config:**
```bash
cat ~/Tank_projects/tank_ws/src/tank_control/config/twist_mux.yaml
```

### Nav2 Not Taking Over After Gamepad Release

**Increase gamepad timeout:**
```yaml
# In twist_mux.yaml
gamepad:
  timeout: 0.2  # Make shorter (was 0.5)
```

**Verify Nav2 is publishing:**
```bash
ros2 topic hz /cmd_vel_nav
```

### Gamepad Lag

**Check network latency:**
```bash
ping <jetson-ip>
```

**Use wired Ethernet** instead of WiFi.

**Reduce joy rate:**
```python
'autorepeat_rate': 10.0,  # Lower from 20.0
```

### ODrive Not Responding

**Check cmd_vel is reaching ODrive:**
```bash
ros2 topic echo /cmd_vel
```

**Verify twist_mux output:**
```bash
rqt_graph  # Check /cmd_vel connection
```

---

## Network Setup (Laptop ↔ Jetson)

### On Both Machines

Add to `~/.bashrc`:

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

Then:
```bash
source ~/.bashrc
```

### Firewall (if needed)

On Jetson:
```bash
sudo ufw allow from <laptop-ip>
# Or disable for testing:
sudo ufw disable
```

---

## Advanced: Multiple Gamepads

Edit `twist_mux.yaml`:

```yaml
gamepad_1:
  topic: /cmd_vel_joy_pilot
  timeout: 0.5
  priority: 110  # Pilot has highest manual priority
  
gamepad_2:
  topic: /cmd_vel_joy_copilot
  timeout: 0.5
  priority: 105  # Co-pilot lower priority
```

---

## Integration with Full System

The `full_autonomy.launch.py` already includes twist_mux via `motor_control.launch.py`.

**No changes needed** - just launch gamepad on laptop!

```bash
# On Jetson
ros2 launch tank_bringup full_autonomy.launch.py

# On Laptop
ros2 launch remote_gamepad.launch.py  # (create this file as shown above)
```

---

## Topic Flow Diagram

```
Laptop Gamepad
    │
    ├─► /joy ──► teleop_twist_joy ──► /cmd_vel_joy ──┐
    │                                                 │
Jetson Nav2                                          │
    │                                                 │
    └─► /cmd_vel_nav ────────────────────────────────┤
                                                      │
                                                      ▼
                                              ┌──────────────┐
                                              │  Twist Mux   │
                                              │  (Priority)  │
                                              └──────┬───────┘
                                                     │
                                                     ▼
                                                 /cmd_vel
                                                     │
                                                     ▼
                                            ┌────────────────┐
                                            │ ODrive + Tank  │
                                            └────────────────┘
```

---

## Checklist

- [ ] `twist_mux` installed
- [ ] `topic_tools` installed
- [ ] `twist_mux.yaml` configured
- [ ] `motor_control.launch.py` includes twist_mux
- [ ] Gamepad connected to laptop
- [ ] `ROS_DOMAIN_ID=42` set on both machines
- [ ] Full system launches without errors
- [ ] Gamepad can override Nav2
- [ ] Nav2 resumes after gamepad timeout

---

**Last Updated:** Dec 19, 2025  
**Status:** Ready for testing

**Next Step:** Test autonomous navigation with gamepad override! 🎮

