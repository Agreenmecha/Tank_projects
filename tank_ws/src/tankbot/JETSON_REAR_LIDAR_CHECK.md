# Jetson: Check Rear LiDAR

## Quick Checklist

**On Jetson, check:**

1. **Physical Connection**
   - Rear LiDAR is powered on
   - Network cable connected
   - LiDAR LED indicators showing activity

2. **Network Configuration**
   - Rear LiDAR IP: `192.168.123.124`
   - Jetson can ping the LiDAR:
     ```bash
     ping 192.168.123.124
     ```

3. **Check Topic Data**
   ```bash
   export ROS_DOMAIN_ID=42
   source /opt/ros/humble/setup.bash
   source ~/Tank_projects/tank_ws/install/setup.bash
   
   # Check if point cloud has data
   ros2 topic echo /lidar_rear/cloud --once | grep width
   # Should show width > 0 (e.g., width: 5000)
   # If width: 0, LiDAR isn't sending data
   ```

4. **Check Node Status**
   ```bash
   ros2 node list | grep lidar_rear
   ros2 node info /lidar_rear/lidar_rear
   ```

5. **Check for Errors**
   - Look at sensor launch output for connection errors
   - Check if rear LiDAR driver is connecting

## Expected Result

After fixing on Jetson:
- `/lidar_rear/cloud` should publish with `width > 0`
- Point cloud should display in RViz on local machine
- Frame ID should be `lidar_rear`

## Once Fixed

On local machine, the rear point cloud should appear in RViz when:
- Fixed Frame: `lidar_rear`
- PointCloud2 topic: `/lidar_rear/cloud`

