---
description: "Standard development workflows: edit, commit, test, deploy patterns"
alwaysApply: false
---

# Development Workflow

## Standard Development Cycle

### 1. Make Changes on Desktop
Edit code in Cursor on Ubuntu Desktop at `/home/aaron/Tank_projects/`

### 2. Test Locally (if possible)
```bash
cd /home/aaron/Tank_projects/tank_ws
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
colcon build --packages-select <package>
source install/setup.bash
# Test the package
```

### 3. Commit and Push
```bash
cd /home/aaron/Tank_projects
git add <files>
git commit -m "Descriptive message"
git push
```

### 4. Deploy to Rover
```bash
cd /home/aaron/Tank_projects/scripts

# Pull changes
./rover exec "cd ~/Tank_projects && git pull"

# Rebuild if needed
./rover session-create rebuild --command \
  "cd ~/Tank_projects/tank_ws && colcon build --packages-select <package>"

# Monitor build
./rover session-output rebuild

# Clean up
./rover session-kill rebuild
```

### 5. Test on Rover
```bash
# Restart services
./rover session-kill sensors  # If already running
./rover ros-launch sensors tank_sensors hardware.launch.py

# Verify
./rover ros-nodes
./rover ros-topics
./rover session-output sensors --lines 50
```

### 6. Visualize on Desktop
```bash
cd /home/aaron/Tank_projects
source tank_ws/install/setup.bash
export ROS_DOMAIN_ID=42
rviz2
```

Configure RViz:
- Fixed Frame: `base_link`
- Add PointCloud2: `/lidar_front/cloud`, `/lidar_rear/cloud`
- Add TF display
- Add RobotModel: `/robot_description`

## Quick Hotfix Pattern

For quick tests without committing:

```bash
# Edit file on desktop
# Test directly on rover without commit
./rover exec "cat > ~/temp_test.py" < local_file.py
./rover exec "python3 ~/temp_test.py"
./rover exec "rm ~/temp_test.py"
```

## Debugging Workflow

### 1. Check System Health
```bash
./rover stats
```

### 2. Check What's Running
```bash
./rover ros-nodes
./rover ps --filter "ros2"
```

### 3. View Logs
```bash
./rover session-output sensors --lines 200 | grep -i error
```

### 4. Check Topics
```bash
./rover ros-topics
./rover ros-hz /lidar_front/cloud
```

### 5. Interactive Debug
```bash
# Start interactive shell on rover
./rover exec "tmux new-session -d -s debug && tmux send-keys -t debug 'bash' Enter"
./rover exec "tmux attach -t debug"  # Won't work via SSH
# Better: use multiple rover exec commands
```

## Testing Sensors

### LiDAR Test
```bash
cd /home/aaron/Tank_projects
source tank_ws/install/setup.bash
export ROS_DOMAIN_ID=42

# Check topics exist
ros2 topic list | grep lidar

# Check rates
ros2 topic hz /lidar_front/cloud
ros2 topic hz /lidar_rear/cloud

# Echo sample
ros2 topic echo /lidar_front/cloud --once
```

### GNSS Test
```bash
# Check GNSS publishing
ros2 topic hz /ubx_nav_pvt
ros2 topic echo /ubx_nav_pvt --once
```

## Common Issues & Solutions

### Build Fails with Missing Directory
```bash
# Create missing directories
./rover exec "mkdir -p ~/Tank_projects/tank_ws/src/<package>/launch"
./rover exec "mkdir -p ~/Tank_projects/tank_ws/src/<package>/config"
```

### ROS_DOMAIN_ID Mismatch
```bash
# Desktop
echo $ROS_DOMAIN_ID  # Should be 42
export ROS_DOMAIN_ID=42

# Rover
./rover exec "echo \$ROS_DOMAIN_ID"  # Should be 42
```

### Sensors Not Starting
```bash
# Check if already running
./rover ps --filter "ros2 launch"

# Kill old processes
./rover session-kill sensors

# Restart
./rover ros-launch sensors tank_sensors hardware.launch.py
```

### Can't See Topics on Desktop
```bash
# Verify network connectivity
ping 192.168.2.100

# Check ROS_DOMAIN_ID matches
echo $ROS_DOMAIN_ID

# Check router/firewall isn't blocking DDS
```

## Git Best Practices

### Commit Messages
```bash
# Good examples
git commit -m "Add obstacle detection node"
git commit -m "Fix LiDAR transform issue"
git commit -m "Update hardware launch with camera support"

# Include context
git commit -m "Refactor sensor drivers

- Split hardware.launch.py for modularity
- Add parameter validation
- Update documentation"
```

### Before Committing
```bash
# Check status
git status

# Review changes
git diff

# Stage selectively
git add specific_files

# Don't commit
# - rover_config.json
# - build/, install/, log/
# - *.pyc, __pycache__/
```

## Performance Monitoring

### Rover Resources
```bash
./rover stats  # Regular monitoring
```

Watch for:
- CPU > 80%
- Temperature > 70°C
- Memory usage high
- Disk space low

### ROS Performance
```bash
# Topic rates
./rover ros-hz /lidar_front/cloud  # Should be ~10Hz

# Check for dropped messages
ros2 topic echo /rosout | grep -i warn
```

## Safety Checks Before Testing

1. Check power connections
2. Verify emergency stop accessible
3. Clear area around rover
4. Test communications before movement
5. Start with low-speed tests

@scripts/ROVER_SSH_README.md

