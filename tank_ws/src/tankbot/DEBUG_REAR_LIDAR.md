# Debug: Rear LiDAR Not Showing in RViz

## Checklist

The topic `/lidar_rear/cloud` exists and is publishing. If you don't see it in RViz:

### 1. Verify Topic in RViz
- PointCloud2 Display → Topic field should be: `/lidar_rear/cloud`
- Make sure there are no typos or extra spaces

### 2. Check Display is Enabled
- PointCloud2 display checkbox must be CHECKED (enabled)
- The display name should not be grayed out

### 3. Check Fixed Frame Matches
- Fixed Frame should be: `lidar_rear` (exact match, case-sensitive)
- Or try: `lidar_rear` (type it manually, don't rely on dropdown)

### 4. Check Point Size
- PointCloud2 → Size (Pixels): Try 3-5 (might be too small to see)
- Style: Try "Flat Squares" or "Spheres" instead of "Points"

### 5. Check Color Settings
- Color: Try "Intensity" or "Z-axis" 
- Make sure it's not set to a color that matches the background

### 6. Check View Settings
- Make sure you're not looking at the wrong angle
- Try resetting the view: Views → Current View → Reset
- Try zooming out (scroll wheel)

### 7. Verify Data is Actually There
```bash
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source ~/Tank_projects/tank_ws/install/setup.bash

# Check if point cloud has data
ros2 topic echo /lidar_rear/cloud --once | grep -E "width|height|data:"
```

Should show width > 0 and data array with values.

### 8. Compare with Front LiDAR
- Does the front LiDAR display work?
- If yes, copy those exact settings for the rear
- Only difference should be the topic name

### 9. Try Different Fixed Frame
- Try Fixed Frame: `base_link` or `world`
- Then check if transform error appears (this tells us if frame_id is different)

### 10. Check RViz Status Bar
- Look at bottom status bar for errors
- Check if it says "No messages received" or transform errors

## Quick Test

Try this exact setup:
1. Fixed Frame: `lidar_rear` (type manually)
2. Add PointCloud2
3. Topic: `/lidar_rear/cloud`
4. Size: 5 pixels
5. Style: Flat Squares
6. Color: Z-axis
7. Enable checkbox: CHECKED

If still nothing, the point cloud might be empty or the frame_id doesn't match.

