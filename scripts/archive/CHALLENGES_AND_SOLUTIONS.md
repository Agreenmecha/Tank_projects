# Rover SSH Bridge - Challenges Encountered & Solutions

## Summary of Task Challenges

When completing the "launch RViz on desktop and sensors on rover" task, these were the main obstacles:

### 1. **Background Process Death** ⚠️

**What happened:**
```bash
# This command would start, then immediately die:
./rover_ssh_wrapper.sh exec "ros2 launch tank_sensors hardware.launch.py &"
```

**Why:** SSH sessions close when the command completes. Background processes started with `&` are killed when the parent SSH session terminates.

**Workaround used:** Realized sensors were already running from a previous session.

**V2 Solution:** 
```bash
# Persistent tmux session survives SSH disconnection
./rover_ssh_bridge_v2.py ros-launch sensors tank_sensors hardware.launch.py
```

---

### 2. **Environment Not Persisting** ⚠️

**What happened:**
```bash
# Every single command needed full environment setup
./rover_ssh_wrapper.sh exec "export ROS_DOMAIN_ID=42 && source ~/Tank_projects/tank_ws/install/setup.bash && ros2 topic list"
```

**Why:** Each `exec` call creates a fresh shell with no memory of previous commands.

**Workaround used:** Repeated the environment setup in every command (tedious but worked).

**V2 Solution:**
```bash
# Automatic ROS environment setup
./rover_ssh_bridge_v2.py ros-topics  # Just works!
```

---

### 3. **No Process Visibility** ⚠️

**What happened:**
- Couldn't tell if sensors were running
- No way to see logs/errors
- Had to manually SSH to check

**Why:** V1 has no process monitoring or session management.

**Workaround used:** Used `ps` commands via exec and checked topics from desktop.

**V2 Solution:**
```bash
# Built-in monitoring
./rover_ssh_bridge_v2.py ps --filter "ros2"
./rover_ssh_bridge_v2.py ros-nodes
./rover_ssh_bridge_v2.py stats
```

---

### 4. **Build Errors** ⚠️

**What happened:**
```
CMake Error: can't find '/home/aaronjet/Tank_projects/tank_ws/src/tank_description/config'
```

**Why:** Missing directories that CMakeLists.txt expected.

**Solution used:** Created missing directories:
```bash
./rover_ssh_wrapper.sh exec "mkdir -p ~/Tank_projects/tank_ws/src/tank_description/config"
```

**Prevention:** Update CMakeLists.txt to not require empty directories, or add `.gitkeep` files.

---

### 5. **Long Command Timeouts** ⚠️

**What happened:**
```bash
# Colcon build takes 60+ seconds, would timeout
./rover_ssh_wrapper.sh exec "colcon build"  # Timeout!
```

**Solution used:** Added `--timeout 180` parameter to allow long-running commands.

**V2 Improvement:** Sessions can run indefinitely:
```bash
./rover_ssh_bridge_v2.py session-create build --command "cd ~/Tank_projects/tank_ws && colcon build"
./rover_ssh_bridge_v2.py session-output build  # Check progress
```

---

## Key Improvements in V2

### Before (V1):
```bash
# Manual environment setup every time
./rover_ssh_wrapper.sh exec "export ROS_DOMAIN_ID=42 && source ~/Tank_projects/tank_ws/install/setup.bash && ros2 launch tank_sensors hardware.launch.py"

# Can't monitor it
# Can't stop it cleanly
# Dies when SSH closes
```

### After (V2):
```bash
# Start
./rover_ssh_bridge_v2.py ros-launch sensors tank_sensors hardware.launch.py

# Monitor
./rover_ssh_bridge_v2.py session-output sensors
./rover_ssh_bridge_v2.py ros-nodes
./rover_ssh_bridge_v2.py stats

# Stop
./rover_ssh_bridge_v2.py session-kill sensors
```

---

## Real-World Example: Today's Task

### How it actually went (with workarounds):

1. ❌ Tried to launch sensors via SSH → Failed (process died)
2. ✅ Found sensors already running from previous session
3. ✅ Verified topics visible from desktop
4. ✅ Launched RViz locally
5. ✅ Success - visualization working!

### How it would go with V2:

1. `./rover_ssh_bridge_v2.py ros-launch sensors tank_sensors hardware.launch.py`
2. `./rover_ssh_bridge_v2.py ros-topics` → Verify topics
3. Launch RViz locally
4. Success!

And if there were issues:
- `./rover_ssh_bridge_v2.py session-output sensors` → Check for errors
- `./rover_ssh_bridge_v2.py stats` → Check system resources
- `./rover_ssh_bridge_v2.py session-kill sensors` → Restart cleanly

---

## Additional V2 Features

### System Monitoring
```bash
$ ./rover_ssh_bridge_v2.py stats
=== Rover System Stats ===
CPU:         29.4%
Memory:      2.3Gi/7.4Gi
Disk:        25G/114G (23%)
Temperature: 54.0°C
Uptime:      up 54 minutes
```

### ROS Introspection
```bash
$ ./rover_ssh_bridge_v2.py ros-hz /lidar_front/cloud
/lidar_front/cloud: 9.78 Hz
```

### Process Management
```bash
$ ./rover_ssh_bridge_v2.py ps --filter "lidar"
 18696  21.2%   0.3% /home/aaronjet/Tank_projects/tank_ws/install/unitree_lidar_ros2/lib/unitree_lidar_ros2/unitree_lidar_ros2_node
 18698  21.1%   0.3% /home/aaronjet/Tank_projects/tank_ws/install/unitree_lidar_ros2/lib/unitree_lidar_ros2/unitree_lidar_ros2_node
```

---

## Recommendations

1. **Use V2 for ROS operations** - Better environment handling, session management
2. **Use V1 for simple commands** - Quick file operations, simple exec
3. **Use sessions for services** - Any long-running process (launches, builds, monitoring)
4. **Monitor regularly** - `stats`, `ros-nodes`, `session-output` to catch issues early

---

## Testing V2

```bash
cd /home/aaron/Tank_projects/scripts

# Test basic connectivity
./rover_ssh_bridge_v2.py stats

# Test ROS commands
./rover_ssh_bridge_v2.py ros-topics
./rover_ssh_bridge_v2.py ros-nodes

# Test process listing
./rover_ssh_bridge_v2.py ps --filter "ros2"

# Test session management (when you're ready to try)
./rover_ssh_bridge_v2.py session-create test --command "echo 'Hello from rover'; sleep 5; echo 'Done'"
./rover_ssh_bridge_v2.py session-output test
./rover_ssh_bridge_v2.py session-kill test
```

