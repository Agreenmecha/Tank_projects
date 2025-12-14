# Fixing LiDAR Point Cloud Frame Issues

## Problem
PointCloud messages are being dropped because they use frame_id='unilidar', but the TF tree expects 'lidar_front' and 'lidar_rear'.

## Solution: Restart Launch File

The `remote_viz.launch.py` now includes a static transform from 'unilidar' to 'lidar_front'. 

**Restart your visualization:**

```bash
# Kill existing launch
pkill -f "remote_viz\|robot_state_publisher"

# Relaunch
cd ~/Tank_projects/tank_ws
source install/setup.bash
ros2 launch tankbot remote_viz.launch.py
```

## Alternative: Fix on Jetson (Better Long-term Solution)

The proper fix is to ensure the LiDAR driver on the Jetson publishes with the correct frame_id. 

**Check current frame_id:**
```bash
# SSH to Jetson
ros2 topic echo /lidar_front/cloud --once | grep frame_id
```

**If it shows 'unilidar', you may need to:**
1. Check the LiDAR launch file parameters
2. Verify the driver respects the `frame_id` parameter
3. Update the launch file if needed

## Verify Fix

After restarting:
1. Check TF frames: `ros2 run tf2_tools view_frames`
2. In RViz, TF display should show 'unilidar' → 'lidar_front' transform
3. PointCloud2 display should show points (not 0 points)
4. Terminal should stop showing "queue is full" errors

## If Still Not Working

If rear LiDAR also uses 'unilidar', you may need separate transforms or namespace handling. Check:

```bash
ros2 topic echo /lidar_rear/cloud --once | grep frame_id
```

If both use 'unilidar', the driver configuration on Jetson needs to be fixed.

