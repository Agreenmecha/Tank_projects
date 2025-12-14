# Jetson Restart Instructions

## Do You Need to Restart on Jetson?

**Short answer: Only if the LiDAR frame IDs are wrong.**

The Jetson only publishes sensor data (point clouds). The local machine publishes the robot TF transforms (via `robot_state_publisher`).

## When to Restart Jetson Sensors

### ✅ RESTART Jetson if:
1. Point cloud frame_id is wrong (check with commands below)
2. You updated `lidar_dual.launch.py` on the Jetson
3. LiDAR point clouds show wrong frame_id

### ❌ DON'T restart Jetson if:
1. Only TF transform errors (those are local machine issue)
2. Robot model not showing in RViz
3. "No transform to world" errors (local machine issue)

## Check Current Frame IDs

**On your local machine:**
```bash
# Check front LiDAR frame_id
ros2 topic echo /lidar_front/pointcloud --once | grep frame_id

# Check rear LiDAR frame_id  
ros2 topic echo /lidar_rear/pointcloud --once | grep frame_id
```

**Should show:**
- Front: `frame_id: "lidar_front"`
- Rear: `frame_id: "lidar_rear"`

## Restart Jetson Sensors (if needed)

**On Jetson (SSH):**
```bash
# Kill existing sensors
pkill -f "lidar|unitree|gnss"

# Relaunch
cd ~/Tank_projects/tank_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tank_sensors hardware.launch.py
```

## Verify Fix

**On local machine:**
```bash
# After restarting Jetson sensors, check frame_ids again
ros2 topic echo /lidar_front/pointcloud --once | grep frame_id
ros2 topic echo /lidar_rear/pointcloud --once | grep frame_id

# Check transforms
ros2 run tf2_ros tf2_echo world lidar_front
ros2 run tf2_ros tf2_echo world lidar_rear
```

## Current Issue: Transform Errors

For your current issue (transform errors), you do **NOT** need to restart Jetson. The problem is on your **local machine** - `robot_state_publisher` needs to be running.

**Fix on local machine:**
```bash
cd ~/Tank_projects/tank_ws
bash src/tankbot/scripts/clean_start_viz.sh
```

This will start `robot_state_publisher` which publishes the URDF transforms (base_link -> l_FL2 -> lidar_front).

