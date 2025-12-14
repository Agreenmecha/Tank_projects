# View Raw LiDAR Point Cloud

## Simple Approach - No Transforms Needed

To view the raw point cloud from the front LiDAR without worrying about transforms:

### Step 1: Launch RViz (Simple)

```bash
cd ~/Tank_projects/tank_ws
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run rviz2 rviz2
```

### Step 2: Configure RViz

1. **Set Fixed Frame to `lidar_front`**
   - Displays → Global Options → Fixed Frame: `lidar_front`
   - This way, the point cloud will display in its own frame (no transforms needed)

2. **Add PointCloud2 Display**
   - Click "Add" button
   - Select "PointCloud2"
   - Set Topic to `/lidar_front/cloud`
   - Set Size (Pixels) to 1-2 for better visibility
   - Color: Intensity or Z-axis

3. **View the Point Cloud**
   - You should now see the raw point cloud data
   - Use mouse to rotate/pan/zoom

### Alternative: Use pcl_ros tools (if available)

```bash
# Install pcl_ros tools (optional)
sudo apt install ros-humble-pcl-ros

# View point cloud in rviz with specific topic
ros2 run rviz2 rviz2 -d <config_file>
```

### Verify Point Cloud is Publishing

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source ~/Tank_projects/tank_ws/install/setup.bash

# Check topic exists
ros2 topic list | grep lidar

# Check message rate
ros2 topic hz /lidar_front/cloud

# Check frame_id in message
ros2 topic echo /lidar_front/cloud --once | grep frame_id

# Should show: frame_id: lidar_front
```

### RViz Configuration Tips

- **Fixed Frame**: Set to `lidar_front` (the frame_id in the point cloud message)
- **PointCloud2 Display**: 
  - Topic: `/lidar_front/cloud`
  - Size: 1-3 pixels
  - Style: Points, Flat Squares, or Spheres
  - Color: Intensity (if available) or Z-axis height
- **View**: Use Orbit view to rotate around the point cloud

### Troubleshooting

If you don't see the point cloud:
1. Check topic is publishing: `ros2 topic hz /lidar_front/cloud`
2. Verify Fixed Frame matches frame_id: `ros2 topic echo /lidar_front/cloud --once | grep frame_id`
3. Check PointCloud2 display is enabled (checkbox checked)
4. Try changing Point Size in the display settings

