# Quick Fix: Transform Errors

## Problem
`lidar_front` has "no transform to world" error.

## Root Cause
`robot_state_publisher` is not running on your **local machine**. The Jetson only publishes sensor data; your local machine must publish the robot TF transforms.

## Solution (Local Machine Only)

**Step 1: Run the clean start script**
```bash
cd ~/Tank_projects/tank_ws
bash src/tankbot/scripts/clean_start_viz.sh
```

**Step 2: Wait 5 seconds for nodes to start, then verify**
```bash
# In a new terminal
source /opt/ros/humble/setup.bash
source ~/Tank_projects/tank_ws/install/setup.bash

# Check nodes are running
ros2 node list | grep -E "(robot_state|joint_state|world_to_base)"

# Should see:
# /robot_state_publisher
# /simple_joint_state_publisher
# /world_to_base_footprint
```

**Step 3: Verify transforms exist**
```bash
ros2 run tf2_ros tf2_echo world lidar_front
ros2 run tf2_ros tf2_echo world lidar_rear
```

Both should show transform data (not "frame does not exist").

## Do You Need to Restart Jetson?

**NO** - Jetson restart is NOT needed for transform errors.

Only restart Jetson if:
- Point cloud frame_id is wrong (check with `ros2 topic echo /lidar_front/pointcloud --once | grep frame_id`)
- You updated the LiDAR launch file on Jetson

## If Still Not Working

If transforms still don't work after clean restart:

1. **Check robot_state_publisher is actually running:**
   ```bash
   ps aux | grep robot_state_publisher
   ```

2. **Check URDF is valid:**
   ```bash
   check_urdf install/tankbot/share/tankbot/urdf/tankbot.urdf
   ```

3. **Check /tf_static topic:**
   ```bash
   ros2 topic echo /tf_static --once | grep -E "(base_link|lidar_front|lidar_rear)"
   ```

4. **Rebuild package:**
   ```bash
   cd ~/Tank_projects/tank_ws
   colcon build --packages-select tankbot
   source install/setup.bash
   ```

