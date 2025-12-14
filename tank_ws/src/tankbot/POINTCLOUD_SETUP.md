# Adding LiDAR Point Clouds to RViz

## Step-by-Step Instructions

### 1. Verify Topics Are Available

First, check that pointcloud topics exist:

```bash
# In terminal (with ROS_DOMAIN_ID=42 set)
ros2 topic list | grep -i "cloud\|point\|lidar"
```

**Expected topics:**
- `/lidar_front/pointcloud` or `/lidar_front/cloud`
- `/lidar_rear/pointcloud` or `/lidar_rear/cloud`

### 2. Add PointCloud Display in RViz

**Method 1: By Topic (Recommended)**
1. In RViz, click **"Add"** button (bottom left)
2. Select **"By topic"** tab
3. Look for topics containing:
   - `cloud`
   - `pointcloud`
   - `lidar_front`
   - `lidar_rear`
4. Click on the pointcloud topic
5. Select **"PointCloud2"** from the type dropdown
6. Click **"OK"**

**Method 2: By Display Type**
1. Click **"Add"** button
2. Select **"By display type"** tab
3. Scroll down and select **"PointCloud2"**
4. Click **"OK"**
5. In the display properties, set:
   - **Topic**: `/lidar_front/pointcloud` (or whatever topic name you see)
   - **Size (m)**: `0.02`
   - **Style**: `Flat Squares`

### 3. Configure PointCloud Display

Once added, expand the PointCloud2 display and configure:

**Basic Settings:**
- **Topic**: Should show the pointcloud topic name
- **Size (m)**: `0.02` to `0.05` (adjust based on point size you want)
- **Style**: `Flat Squares` or `Points`
- **Decay Time**: `10` seconds (how long points stay visible)

**Color Settings:**
- **Color Transformer**: 
  - `Intensity` - Colors by LiDAR return intensity (best for most cases)
  - `Flat Color` - Single color (useful for rear LiDAR to differentiate)
- **Color**: Choose color if using Flat Color
- **Channel Name**: `intensity` (if using Intensity transformer)

### 4. Add Second LiDAR

Repeat steps 2-3 for the rear LiDAR:
- Topic: `/lidar_rear/pointcloud`
- Use different color (e.g., Red) to distinguish from front

---

## Troubleshooting: No Point Cloud Visible

### Issue: PointCloud display added but no points visible

**Check 1: Fixed Frame**
- Set Fixed Frame to `world` or `base_footprint`
- Points might be far from origin if frame is wrong

**Check 2: Topic is publishing**
```bash
ros2 topic hz /lidar_front/pointcloud
# Should show ~10 Hz if publishing
```

**Check 3: Topic has data**
```bash
ros2 topic echo /lidar_front/pointcloud --once
# Should show pointcloud data
```

**Check 4: Camera view**
- Try zooming out/in in RViz
- Use "Focus Camera" tool to center on robot
- Points might be very small or very large

**Check 5: Point size**
- Increase "Size (m)" in PointCloud display
- Try `0.05` or `0.1` to make points more visible

### Issue: Topics not showing in RViz "Add" dialog

**Check 1: Topics exist**
```bash
ros2 topic list
# Should list all available topics
```

**Check 2: Sensors running on Jetson**
```bash
# SSH to Jetson and check:
ssh aaronjet@<jetson_ip>
ros2 node list
# Should show: /lidar_front, /lidar_rear, etc.
```

**Check 3: ROS_DOMAIN_ID matches**
```bash
# On laptop
echo $ROS_DOMAIN_ID  # Should show: 42

# On Jetson (SSH)
echo $ROS_DOMAIN_ID  # Should also show: 42
```

**Check 4: Network connectivity**
```bash
# From laptop
ping <jetson_ip>
# Should get responses
```

---

## Expected Result

When working correctly, you should see:
- **Robot model**: Tankbot chassis, tracks, sensors
- **Front LiDAR points**: Colored by intensity, showing forward scan
- **Rear LiDAR points**: Different color, showing rear scan
- **Points updating**: Cloud refreshes ~10 times per second

---

## Quick Checklist

- [ ] Sensors launched on Jetson
- [ ] ROS_DOMAIN_ID=42 on both machines
- [ ] Topics visible: `ros2 topic list` shows pointcloud topics
- [ ] Fixed Frame set to `world` or `base_footprint` in RViz
- [ ] PointCloud2 display added in RViz
- [ ] PointCloud topic set correctly
- [ ] Size (m) set to 0.02-0.05
- [ ] Style set to Flat Squares
- [ ] Camera view shows robot (use Focus Camera if needed)

