# Rover SSH Bridge - Quick Reference Card

## Essential Commands

```bash
cd /home/aaron/Tank_projects/scripts

# System Health
./rover stats                           # CPU, memory, temp, uptime

# ROS Status
./rover ros-topics                      # List all topics
./rover ros-nodes                       # List all nodes
./rover ros-hz /lidar_front/cloud      # Check topic rate

# Start Sensors
./rover ros-launch sensors tank_sensors hardware.launch.py

# Monitor Sensors
./rover session-output sensors          # View logs
./rover session-list                    # List all sessions

# Stop Sensors
./rover session-kill sensors

# Process Management
./rover ps --filter "ros2"              # Find ROS processes
./rover kill <pid>                      # Kill process

# Execute Any Command
./rover exec "command here"

# Git Sync
git push                                # From desktop
./rover exec "cd ~/Tank_projects && git pull"  # To rover
```

## Common Workflows

### Startup
```bash
./rover stats                           # Check health
./rover ros-launch sensors tank_sensors hardware.launch.py
./rover ros-nodes                       # Verify running
```

### Monitoring
```bash
./rover session-output sensors --lines 50
./rover ros-hz /lidar_front/cloud
./rover stats
```

### Debugging
```bash
./rover session-output sensors | grep -i error
./rover ps --filter "ros2"
./rover stats
```

### Shutdown
```bash
./rover session-kill sensors
```

## Configuration
- **IP:** 192.168.2.100
- **User:** aaronjet  
- **ROS_DOMAIN_ID:** 42
- **Config:** `scripts/rover_config.json`

## Help
```bash
./rover --help                          # Main help
./rover <command> --help                # Command help
```

## Documentation
- **Complete Guide:** `ROVER_SSH_README.md`
- **Improvements:** `ROVER_SSH_V2_IMPROVEMENTS.md`
- **Troubleshooting:** `CHALLENGES_AND_SOLUTIONS.md`

