# Rover SSH Bridge - Complete Guide

Simple, powerful SSH access to your rover from Cursor AI and the terminal.

## Quick Start

```bash
cd /home/aaron/Tank_projects/scripts

# Test connection
./rover stats

# List ROS topics
./rover ros-topics

# Start sensors
./rover ros-launch sensors tank_sensors hardware.launch.py

# Monitor sensors
./rover session-output sensors
```

---

## Installation & Setup

### 1. Already Done! ✓
- SSH bridge installed
- Configuration set up in `rover_config.json`
- Connection tested and working

### 2. Optional: Add to PATH
```bash
# Add to ~/.bashrc for system-wide access
echo 'export PATH="$PATH:/home/aaron/Tank_projects/scripts"' >> ~/.bashrc
source ~/.bashrc

# Now you can use "rover" from anywhere!
rover stats
```

---

## Command Reference

### System Monitoring

```bash
# Get system stats (CPU, memory, temperature)
./rover stats

# List processes
./rover ps

# Filter processes
./rover ps --filter "ros2"
./rover ps --filter "python"
```

### ROS Operations

```bash
# List topics
./rover ros-topics

# List nodes
./rover ros-nodes

# Check topic rate
./rover ros-hz /lidar_front/cloud

# Launch a ROS package
./rover ros-launch <session_name> <package> <launch_file>
./rover ros-launch sensors tank_sensors hardware.launch.py

# Launch with arguments
./rover ros-launch sensors tank_sensors hardware.launch.py --args "enable_camera:=false enable_gnss:=true"

# Run a ROS node
./rover ros-node <session_name> <package> <executable>
```

### Session Management

Sessions are persistent terminal sessions that survive disconnections.

```bash
# Create a session
./rover session-create <name> --command "your command here"

# Create session without ROS environment
./rover session-create <name> --command "htop" --no-ros

# List all sessions
./rover session-list

# View session output
./rover session-output <name>

# View more lines
./rover session-output <name> --lines 100

# Kill a session
./rover session-kill <name>
```

### File Operations

```bash
# Read a file (from Python API)
from scripts.rover_ssh_bridge_v2 import RoverSSHBridgeV2
bridge = RoverSSHBridgeV2()
bridge.connect()
content = bridge.read_file("/home/aaronjet/config.yaml")
bridge.disconnect()
```

### Process Management

```bash
# Kill a process by PID
./rover kill <pid>
```

### Legacy Commands

```bash
# Execute any command (V1 compatibility)
./rover exec "command here"
./rover exec "ls -la ~/Tank_projects"
```

---

## Common Workflows

### Starting All Sensors

```bash
# Start sensors in persistent session
./rover ros-launch sensors tank_sensors hardware.launch.py \
  --args "enable_gnss:=true enable_lidars:=true enable_camera:=false"

# Verify they're running
./rover ros-nodes

# Check data flow
./rover ros-hz /lidar_front/cloud
./rover ros-hz /lidar_rear/cloud
```

### Monitoring Sensors

```bash
# View sensor output
./rover session-output sensors --lines 50

# Check system resources
./rover stats

# List all ROS processes
./rover ps --filter "ros2"
```

### Stopping Sensors

```bash
./rover session-kill sensors
```

### Building Workspace

```bash
# Create build session
./rover session-create build --command \
  "cd ~/Tank_projects/tank_ws && colcon build --symlink-install"

# Monitor build progress
watch -n 1 './rover session-output build --lines 30'

# When done
./rover session-kill build
```

### Deploying Code

```bash
# 1. Commit and push from desktop
git add .
git commit -m "Update feature"
git push

# 2. Pull on rover
./rover exec "cd ~/Tank_projects && git pull"

# 3. Rebuild if needed
./rover session-create rebuild --command \
  "cd ~/Tank_projects/tank_ws && colcon build --packages-select my_package"

# 4. Restart nodes
./rover session-kill sensors
./rover ros-launch sensors tank_sensors hardware.launch.py
```

### Debugging

```bash
# Check if nodes are running
./rover ros-nodes

# View recent logs
./rover session-output sensors --lines 100

# Check for errors in ROS topics
./rover exec "source ~/Tank_projects/tank_ws/install/setup.bash && ros2 topic echo /rosout --once"

# Monitor system health
./rover stats
```

---

## Python API

For scripts and automation:

```python
from rover_ssh_bridge_v2 import RoverSSHBridgeV2

# Initialize
bridge = RoverSSHBridgeV2()
bridge.connect()

# ROS operations
topics = bridge.ros_topic_list()
nodes = bridge.ros_node_list()
rate = bridge.ros_topic_hz("/lidar_front/cloud")

# Session management
bridge.ros_launch("sensors", "tank_sensors", "hardware.launch.py",
                  args={"enable_camera": "false"})
output = bridge.tmux_get_output("sensors", lines=50)
bridge.tmux_kill_session("sensors")

# System monitoring
stats = bridge.get_system_stats()
print(f"CPU: {stats['cpu_usage']}, Temp: {stats['temperature']}")

# Process management
processes = bridge.get_process_list("ros2")
for proc in processes:
    print(f"PID {proc['pid']}: {proc['command']}")

# File operations
content = bridge.read_file("/home/aaronjet/config.yaml")
bridge.write_file("/home/aaronjet/test.txt", "content")

# Cleanup
bridge.disconnect()
```

---

## Tips & Best Practices

### 1. Use Sessions for Long-Running Processes
```bash
# ✅ Good - survives disconnection
./rover ros-launch sensors tank_sensors hardware.launch.py

# ❌ Bad - dies when SSH closes
./rover exec "ros2 launch tank_sensors hardware.launch.py &"
```

### 2. Monitor Regularly
```bash
# Quick health check
./rover stats && ./rover ros-nodes
```

### 3. Name Sessions Descriptively
```bash
./rover ros-launch lidar_only tank_sensors lidar_dual.launch.py
./rover ros-launch navigation tank_navigation nav.launch.py
./rover session-create monitor_logs --command "tail -f /var/log/syslog"
```

### 4. Use `session-output` to Debug
```bash
# Something not working? Check the logs
./rover session-output sensors --lines 200 | grep -i error
```

### 5. Clean Up Old Sessions
```bash
./rover session-list
./rover session-kill old_session_name
```

---

## Troubleshooting

### Connection Issues

```bash
# Test basic connectivity
./rover exec "echo 'Connection OK'"

# Check config
cat rover_config.json

# Verify rover is reachable
ping 192.168.2.100
```

### Sensors Not Starting

```bash
# Check if already running
./rover ps --filter "ros2 launch"

# View error logs
./rover session-output sensors

# Kill and restart
./rover session-kill sensors
./rover ros-launch sensors tank_sensors hardware.launch.py
```

### No Topics Visible

```bash
# Check ROS_DOMAIN_ID on desktop
echo $ROS_DOMAIN_ID  # Should be 42

# Check on rover
./rover exec "printenv ROS_DOMAIN_ID"

# List topics from rover
./rover ros-topics
```

### tmux Not Found

```bash
# V2 will auto-install tmux, but if manual install needed:
./rover exec "sudo apt update && sudo apt install -y tmux"
```

---

## Configuration

### rover_config.json

```json
{
  "host": "192.168.2.100",
  "port": 22,
  "username": "aaronjet",
  "password": "your_password",
  "key_file": "~/.ssh/id_rsa",
  "use_key": false,
  "timeout": 10
}
```

**Security Note:** Keep this file secure (chmod 600). Consider using SSH keys instead of passwords.

---

## Migration from V1

V1 commands still work via `exec`:

| V1 | V2 Equivalent |
|----|---------------|
| `./rover_ssh_wrapper.sh exec "ros2 topic list"` | `./rover ros-topics` |
| `./rover_ssh_wrapper.sh exec "ros2 node list"` | `./rover ros-nodes` |
| `./rover_ssh_wrapper.sh exec "ps aux \| grep ros2"` | `./rover ps --filter ros2` |
| Manual background processes | `./rover ros-launch` or `session-create` |

V1 files backed up as:
- `rover_ssh_wrapper_v1_backup.sh`
- `rover_ssh_bridge_v1_backup.py`

---

## Examples

### Example 1: Complete Startup Sequence

```bash
# Check rover status
./rover stats

# Start all sensors
./rover ros-launch sensors tank_sensors hardware.launch.py

# Wait a few seconds, then verify
./rover ros-nodes
./rover ros-hz /lidar_front/cloud

# On desktop: launch RViz
cd ~/Tank_projects
source tank_ws/install/setup.bash
export ROS_DOMAIN_ID=42
rviz2
```

### Example 2: Debug Session

```bash
# Something's wrong, let's investigate
./rover stats  # Check resources
./rover ros-nodes  # Check what's running
./rover session-output sensors --lines 200  # Check logs

# Found the issue, restart sensors
./rover session-kill sensors
./rover ros-launch sensors tank_sensors hardware.launch.py
```

### Example 3: Development Cycle

```bash
# Edit code on desktop in Cursor
# Commit changes
git add . && git commit -m "Fix bug" && git push

# Deploy to rover
./rover exec "cd ~/Tank_projects && git pull"

# Rebuild specific package
./rover session-create build --command \
  "cd ~/Tank_projects/tank_ws && colcon build --packages-select my_package"

# Monitor build
./rover session-output build

# When build completes, restart
./rover session-kill sensors build
./rover ros-launch sensors tank_sensors hardware.launch.py
```

---

## Command Cheat Sheet

```bash
# Most used commands
./rover stats                    # System status
./rover ros-topics              # List topics
./rover ros-nodes               # List nodes
./rover ros-launch <name> <pkg> <launch>  # Start ROS
./rover session-output <name>   # View logs
./rover session-kill <name>     # Stop session
./rover ps --filter "ros2"      # Find processes

# Session workflow
./rover session-create test --command "command"
./rover session-output test
./rover session-kill test

# Quick checks
./rover ros-hz /lidar_front/cloud    # Topic rate
./rover exec "uptime"                 # Quick command
```

---

## Support Files

- **rover_ssh_bridge_v2.py** - Main bridge implementation
- **rover_config.json** - Connection configuration
- **rover** - Simple wrapper script
- **ROVER_SSH_V2_IMPROVEMENTS.md** - Detailed improvements doc
- **CHALLENGES_AND_SOLUTIONS.md** - Problem-solving guide

---

## Quick Reference

**Rover IP:** 192.168.2.100  
**ROS Domain ID:** 42  
**Workspace:** ~/Tank_projects/tank_ws  
**Main launch:** tank_sensors hardware.launch.py  

Ready to control your rover! 🚀

