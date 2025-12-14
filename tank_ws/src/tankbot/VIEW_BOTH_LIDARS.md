# View Both Front and Rear LiDAR Point Clouds

## Both LiDARs Are Publishing

- Front: `/lidar_front/cloud` with `frame_id: lidar_front`
- Rear: `/lidar_rear/cloud` with `frame_id: lidar_rear`

## Option 1: View Both in Separate Displays (Recommended)

Since both point clouds have different frame_ids, you can view them both:

1. **Keep Fixed Frame as `lidar_front`** (or use `base_link` if transforms work)

2. **Add Front LiDAR PointCloud2** (if not already added)
   - Click "Add" → PointCloud2
   - Topic: `/lidar_front/cloud`
   - Size: 2-3 pixels
   - Name it "Front LiDAR" for clarity

3. **Add Rear LiDAR PointCloud2**
   - Click "Add" → PointCloud2
   - Topic: `/lidar_rear/cloud`
   - Size: 2-3 pixels
   - Name it "Rear LiDAR" for clarity
   - Color: Use a different color scheme (e.g., Intensity vs Z-axis) to distinguish from front

4. **Both should display!**
   - They'll appear in their respective frames
   - You'll see both point clouds updating at ~10 Hz each

## Option 2: Use `base_link` as Fixed Frame (If Transforms Work)

If the transform tree is working:

1. Set Fixed Frame to `base_link`
2. Add both PointCloud2 displays
3. Both point clouds will appear in the same coordinate frame (aligned with the robot)

## Option 3: Create Simple Static Transforms (Quick Fix)

If you want both to appear relative to each other:

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source ~/Tank_projects/tank_ws/install/setup.bash

# Create transform from lidar_front to lidar_rear (approximate)
# This connects the two frames so both can be viewed together
ros2 run tf2_ros static_transform_publisher \
  0.6 0 0 0 0 0 \
  lidar_front lidar_rear
```

Then in RViz:
- Set Fixed Frame to `lidar_front`
- Add both PointCloud2 displays
- Both will appear in the same view

## Recommended: Just Add Both Displays

The simplest approach:
1. Fixed Frame: `lidar_front` (or `lidar_rear`, doesn't matter much)
2. Add two PointCloud2 displays - one for each topic
3. Use different colors/sizes to distinguish them
4. Both will display and update independently

Even without perfect transforms, you'll see both point clouds - they just won't be perfectly aligned relative to each other, but you can still see what each LiDAR is detecting.

