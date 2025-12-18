# Rover SSH Bridge V2 - Improvements & Solutions

## Challenges from V1 and Solutions

### Challenge 1: Background Process Management ❌ → ✅

**Problem:**
```bash
# This failed - process died when SSH disconnected
./rover_ssh_wrapper.sh exec "ros2 launch tank_sensors hardware.launch.py &"
```

**Solution in V2:**
```bash
# Persistent tmux session keeps process alive
./rover_ssh_bridge_v2.py ros-launch sensors_session tank_sensors hardware.launch.py

# Check it's still running
./rover_ssh_bridge_v2.py session-list

# View output
./rover_ssh_bridge_v2.py session-output sensors_session
```

**How it works:** Uses tmux (terminal multiplexer) to create persistent sessions that survive SSH disconnections.

---

### Challenge 2: Environment Persistence ❌ → ✅

**Problem:**
```bash
# Had to source ROS in EVERY command
./rover_ssh_wrapper.sh exec "source ~/Tank_projects/tank_ws/install/setup.bash && ros2 topic list"
./rover_ssh_wrapper.sh exec "source ~/Tank_projects/tank_ws/install/setup.bash && ros2 node list"
# Tedious and error-prone!
```

**Solution in V2:**
```bash
# V2 automatically sources ROS environment
./rover_ssh_bridge_v2.py ros-topics
./rover_ssh_bridge_v2.py ros-nodes
./rover_ssh_bridge_v2.py ros-hz /lidar_front/cloud

# Sessions maintain environment
./rover_ssh_bridge_v2.py session-create my_session --command "ros2 topic echo /cmd_vel"
```

**How it works:** `source_ros=True` parameter automatically sets up ROS environment in sessions.

---

### Challenge 3: No Process Monitoring ❌ → ✅

**Problem:**
```bash
# Started sensors... but are they running? No idea!
# Had to manually SSH and check
```

**Solution in V2:**
```bash
# List all ROS nodes
./rover_ssh_bridge_v2.py ros-nodes

# Check topic rates
./rover_ssh_bridge_v2.py ros-hz /lidar_front/cloud
# Output: /lidar_front/cloud: 9.78 Hz

# See all processes
./rover_ssh_bridge_v2.py ps --filter "ros2"

# Get system stats
./rover_ssh_bridge_v2.py stats
```

**How it works:** Built-in ROS introspection and process monitoring commands.

---

### Challenge 4: Can't View Logs ❌ → ✅

**Problem:**
```bash
# Sensors launched in background... but how to see errors?
# No stdout/stderr access
```

**Solution in V2:**
```bash
# Launch in session
./rover_ssh_bridge_v2.py ros-launch sensors tank_sensors hardware.launch.py

# View live output (last 50 lines)
./rover_ssh_bridge_v2.py session-output sensors

# View more lines
./rover_ssh_bridge_v2.py session-output sensors --lines 200

# Continuously monitor (from desktop terminal)
watch -n 1 "./rover_ssh_bridge_v2.py session-output sensors --lines 30"
```

**How it works:** tmux captures all terminal output, retrievable at any time.

---

### Challenge 5: Can't Stop Services ❌ → ✅

**Problem:**
```bash
# Started sensors... now how to stop them?
# Had to find PID manually, kill manually
```

**Solution in V2:**
```bash
# Simple session management
./rover_ssh_bridge_v2.py session-kill sensors

# Or fine-grained process control
./rover_ssh_bridge_v2.py ps --filter "ros2 launch"
# Find PID
./rover_ssh_bridge_v2.py kill 12345
```

**How it works:** Session names provide clean abstraction, or direct process control.

---

## New Capabilities in V2

### 1. System Monitoring
```bash
./rover_ssh_bridge_v2.py stats
```
Output:
```
=== Rover System Stats ===
CPU:         15.2%
Memory:      2.3G/7.6G
Disk:        25G/114G (23%)
Temperature: 45.3°C
Uptime:      up 3 hours, 24 minutes
```

### 2. ROS Introspection
```bash
# List all topics with rates
for topic in $(./rover_ssh_bridge_v2.py ros-topics | grep lidar); do
    ./rover_ssh_bridge_v2.py ros-hz $topic
done
```

### 3. Session Management
```bash
# Create custom session
./rover_ssh_bridge_v2.py session-create debug --command "htop"

# List all sessions
./rover_ssh_bridge_v2.py session-list

# Interact with session
./rover_ssh_bridge_v2.py session-output debug
```

---

## Typical V2 Workflow

### Starting Sensors:
```bash
# V1 (didn't work reliably):
./rover_ssh_wrapper.sh exec "nohup ros2 launch ... &"  # ❌

# V2 (clean and persistent):
./rover_ssh_bridge_v2.py ros-launch sensors tank_sensors hardware.launch.py  # ✅
```

### Checking Status:
```bash
# Verify nodes are running
./rover_ssh_bridge_v2.py ros-nodes

# Check data flow
./rover_ssh_bridge_v2.py ros-hz /lidar_front/cloud

# View any errors
./rover_ssh_bridge_v2.py session-output sensors --lines 100
```

### Stopping:
```bash
# Clean shutdown
./rover_ssh_bridge_v2.py session-kill sensors
```

---

## Python API Improvements

### V1 API:
```python
bridge = RoverSSHBridge()
bridge.connect()

# Had to handle environment manually
exit_code, stdout, stderr = bridge.run_command(
    "source ~/Tank_projects/tank_ws/install/setup.bash && ros2 topic list"
)
# Parse stdout yourself
```

### V2 API:
```python
bridge = RoverSSHBridgeV2()
bridge.connect()

# Clean ROS operations
topics = bridge.ros_topic_list()  # Returns list
nodes = bridge.ros_node_list()    # Returns list
rate = bridge.ros_topic_hz("/lidar_front/cloud")  # Returns float

# Session management
bridge.ros_launch("sensors", "tank_sensors", "hardware.launch.py", 
                  args={"enable_camera": "false"})

# Monitor
output = bridge.tmux_get_output("sensors", lines=50)
stats = bridge.get_system_stats()  # Returns dict

# Process control
processes = bridge.get_process_list("ros2")
bridge.kill_process(processes[0]['pid'])
```

---

## Migration Guide

V1 commands still work in V2! The `exec` command is maintained for compatibility:

```bash
# Old V1 command
./rover_ssh_wrapper.sh exec "command"

# Still works in V2
./rover_ssh_bridge_v2.py exec "command"
```

But you should migrate to new commands for better functionality:

| V1 Pattern | V2 Improvement |
|------------|----------------|
| `exec "ros2 topic list"` | `ros-topics` |
| `exec "ros2 node list"` | `ros-nodes` |
| `exec "ps aux \| grep ros2"` | `ps --filter ros2` |
| Manual background processes | `session-create` / `ros-launch` |
| Manual log viewing | `session-output` |

---

## Installation

```bash
# V2 uses the same config as V1
cd /home/aaron/Tank_projects/scripts
chmod +x rover_ssh_bridge_v2.py

# Test it
./rover_ssh_bridge_v2.py stats
./rover_ssh_bridge_v2.py ros-topics
```

V2 will auto-install tmux on the rover if not present.

---

## Summary

**V1 was good for:**
- Simple command execution ✓
- File operations ✓
- One-off tasks ✓

**V2 adds:**
- Persistent background processes ✓
- Session management ✓
- ROS-specific operations ✓
- Process monitoring ✓
- System stats ✓
- Log viewing ✓
- Clean service lifecycle ✓

**Result:** Can now do complete rover development and operations remotely, including starting/stopping/monitoring long-running ROS nodes.

