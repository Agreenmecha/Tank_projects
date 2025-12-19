---
description: "Rover SSH access patterns and commands for remote operations"
alwaysApply: false
---

# Rover SSH Access

## Connection Details
- **Host:** 192.168.2.100 (tankviaxiaomi)
- **User:** aaronjet
- **Password:** midang#1 (for sudo operations)
- **ROS_DOMAIN_ID:** 42

## Rover Command Usage

Use the `./scripts/rover` command for all rover operations:

```bash
cd /home/aaron/Tank_projects/scripts
./rover <subcommand> [args]
```

## Common Commands

### System Monitoring
```bash
./rover stats                           # CPU, memory, temperature, uptime
./rover ps --filter "ros2"             # Find ROS processes
```

### ROS Operations
```bash
./rover ros-topics                      # List all topics
./rover ros-nodes                       # List all nodes
./rover ros-hz /topic/name             # Check topic rate
```

### Session Management
```bash
# Start ROS services in persistent session
./rover ros-launch <name> <package> <launch_file>
./rover ros-launch sensors tank_sensors hardware.launch.py

# Monitor session output
./rover session-output <name> --lines 100

# List active sessions
./rover session-list

# Kill session
./rover session-kill <name>
```

### Legacy/Direct Commands
```bash
./rover exec "any bash command"
./rover exec "command" --timeout 120
```

## Git Sync Pattern

```bash
# Desktop → GitHub
git add . && git commit -m "message" && git push

# Rover ← GitHub
./rover exec "cd ~/Tank_projects && git pull"
```

## Starting Sensors

```bash
# Start all sensors
./rover ros-launch sensors tank_sensors hardware.launch.py

# Verify running
./rover ros-nodes
./rover ros-hz /lidar_front/cloud
```

## Common Issues

- **Session not persisting:** Ensure tmux is installed on rover
- **ROS_DOMAIN_ID mismatch:** Check `echo $ROS_DOMAIN_ID` on both systems (should be 42)
- **Timeout on long commands:** Add `--timeout` parameter to exec

## Important Notes

- Sessions survive SSH disconnections
- ROS environment is automatically sourced in sessions
- Use `session-output` to view logs from running processes
- Use direct tmux commands if Python API has issues:
  ```bash
  ./rover exec "tmux new-session -d -s name && tmux send-keys -t name 'command' Enter"
  ```

@scripts/rover
@scripts/ROVER_QUICK_REFERENCE.md

