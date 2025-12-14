# Quick: View Front LiDAR Point Cloud

## Simple Steps (No Transforms Needed)

**The point cloud is publishing with `frame_id: lidar_front` at ~9.7 Hz**

### In RViz:

1. **Set Fixed Frame to `lidar_front`**
   - Left panel: Displays → Global Options
   - Fixed Frame: type `lidar_front` and press Enter
   - This eliminates the need for any transforms!

2. **Add PointCloud2 Display**
   - Click "Add" button (bottom left)
   - Select "PointCloud2"
   - In the new PointCloud2 display settings:
     - Topic: `/lidar_front/cloud`
     - Size (Pixels): 2 or 3 (for better visibility)
     - Style: Points or Flat Squares
     - Decay Time: 0 (to see live data)

3. **You should now see the point cloud!**
   - Use mouse to rotate/pan/zoom
   - Middle mouse button to pan
   - Scroll to zoom

### Verify It's Working

The point cloud should display immediately. You should see:
- ~5000 points per scan
- Updated at ~10 Hz
- Points in 3D space around the LiDAR

### If You Don't See It

1. Check the PointCloud2 display checkbox is checked (enabled)
2. Try increasing Size to 3-5 pixels
3. Verify topic: `ros2 topic echo /lidar_front/cloud --once | grep frame_id`

That's it! No robot model, no transforms - just the raw point cloud.

