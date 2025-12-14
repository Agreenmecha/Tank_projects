# Troubleshoot: Rear LiDAR Not Displaying

## Issue
Front LiDAR works in RViz, but rear LiDAR doesn't show even though topic exists.

## Possible Causes

### 1. Rear LiDAR Not Connected/Powered
The rear LiDAR might not be physically connected or powered on.

**Check on Jetson:**
```bash
# SSH to Jetson
ssh aaronjet@<jetson_ip>

# Check if rear LiDAR node is running
ros2 node list | grep lidar_rear

# Check for connection errors
ros2 topic echo /lidar_rear/cloud --once
# If empty or errors, LiDAR might not be connected
```

### 2. Empty Point Cloud
The rear point cloud might be publishing but with zero points.

**Check:**
```bash
export ROS_DOMAIN_ID=42
ros2 topic echo /lidar_rear/cloud --once | grep -E "width|height"
```

If `width: 0`, the point cloud is empty (LiDAR not seeing anything or not connected).

### 3. Frame ID Mismatch
Check if frame_id matches Fixed Frame in RViz.

**Verify:**
```bash
ros2 topic echo /lidar_rear/cloud --once | grep frame_id
```

Should be `lidar_rear`. In RViz, Fixed Frame must match exactly.

### 4. RViz Display Settings
Even if data exists, RViz might not show it due to:
- Point size too small
- Color matches background
- View angle wrong
- Display not enabled

**Try:**
- Increase point size to 5-10 pixels
- Change color to "Intensity" or "Z-axis"
- Reset view (Views → Reset)
- Make sure PointCloud2 checkbox is checked

### 5. Compare with Front
Since front works, copy its exact settings:
- Same point size
- Same color scheme
- Same style
- Only difference: topic name

## Quick Test

**In RViz:**
1. Fixed Frame: `lidar_rear` (type manually)
2. Add PointCloud2
3. Topic: `/lidar_rear/cloud`
4. Size: 10 pixels (make it big to see)
5. Style: Flat Squares
6. Color: Z-axis (height-based coloring)
7. Enable: CHECKED

If still nothing, the rear LiDAR might not be publishing data (not connected or no points).

## Check Rear LiDAR Status on Jetson

The rear LiDAR might need to be:
- Physically connected to the network
- Powered on
- Configured with correct IP (192.168.123.124)

Check the Jetson sensor logs for rear LiDAR connection errors.

