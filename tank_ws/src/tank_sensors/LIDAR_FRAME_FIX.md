# Fixed LiDAR Frame IDs

## What Was Changed

Updated `lidar_dual.launch.py` to use the correct parameter name:
- Changed `frame_id` → `cloud_frame` (for pointcloud frame)
- Added `imu_frame` parameter for IMU frame

The Unitree L2 driver expects:
- `cloud_frame`: Frame ID for pointcloud messages
- `imu_frame`: Frame ID for IMU messages

## To Apply Fix on Jetson

**On the Jetson (rover):**

1. **Rebuild the package:**
   ```bash
   cd ~/Tank_projects/tank_ws
   source /opt/ros/humble/setup.bash
   colcon build --packages-select tank_sensors
   source install/setup.bash
   ```

2. **Restart the sensors:**
   ```bash
   # Kill existing sensor nodes
   pkill -f "lidar\|unitree"
   
   # Relaunch with fixed configuration
   ros2 launch tank_sensors hardware.launch.py
   ```

3. **Verify frame IDs are correct:**
   ```bash
   # Check pointcloud frame_id
   ros2 topic echo /lidar_front/pointcloud --once | grep frame_id
   # Should show: frame_id: "lidar_front"
   
   ros2 topic echo /lidar_rear/pointcloud --once | grep frame_id
   # Should show: frame_id: "lidar_rear"
   ```

## Expected Result

After restarting sensors on Jetson:
- Pointcloud messages will use `lidar_front` and `lidar_rear` frame_ids
- RViz will be able to transform messages correctly
- No more "queue is full" errors
- Point clouds will display aligned with robot model

## Verification

On your local machine:
1. Restart visualization (if running)
2. Check RViz PointCloud2 displays - should show points (not "0 points")
3. Points should align with robot model
4. Terminal should not show "dropping message" errors

