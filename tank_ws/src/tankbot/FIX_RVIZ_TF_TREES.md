# Fix: RViz Showing "Two or More Unconnected Trees"

## Problem
RViz shows "No transform from lidar_front to world" even though all transforms are published correctly.

## Root Cause
All transforms ARE being published correctly. The issue is that RViz's TF buffer is seeing them as separate trees initially because:
1. `world -> base_footprint` is published separately (from static_transform_publisher)
2. All URDF transforms are published together (from robot_state_publisher)
3. RViz may query for transforms before the TF buffer fully connects them

## Solution

**The transforms ARE working - you just need to refresh RViz's TF buffer.**

### Method 1: Restart RViz (Recommended)
1. Close RViz completely
2. Wait 5 seconds
3. Restart using the clean start script:
   ```bash
   cd ~/Tank_projects/tank_ws
   export ROS_DOMAIN_ID=42
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   bash src/tankbot/scripts/clean_start_viz.sh
   ```

### Method 2: Force TF Buffer Refresh in RViz
1. In RViz: **Displays** → **Global Options**
2. Change **Fixed Frame** to `base_link`, wait 3 seconds
3. Change **Fixed Frame** to `base_footprint`, wait 3 seconds  
4. Change **Fixed Frame** back to `world`, wait 5 seconds
5. The transforms should now connect

### Method 3: Verify Transforms Work (Command Line)
Before trying RViz fixes, verify transforms work from command line:
```bash
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source ~/Tank_projects/tank_ws/install/setup.bash

# This should show transform data (may take 2-3 seconds)
ros2 run tf2_ros tf2_echo world lidar_front
ros2 run tf2_ros tf2_echo world lidar_front_imu
```

If command line works but RViz doesn't, it's a RViz TF buffer cache issue.

## Verify All Transforms Are Published

Check that all transforms exist:
```bash
export ROS_DOMAIN_ID=42
ros2 topic echo /tf_static --once | grep -E "frame_id|child_frame_id"
```

Should show:
- `world -> base_footprint`
- `base_footprint -> base_link`
- `base_link -> l_FL2`
- `l_FL2 -> lidar_front`
- `lidar_front -> lidar_front_imu`

## Important: ROS_DOMAIN_ID

Make sure RViz and all nodes are using the same ROS_DOMAIN_ID (42 in your case):
```bash
export ROS_DOMAIN_ID=42
```

Add to `~/.bashrc` for persistence:
```bash
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
```

## Why This Happens

Static transforms are published as separate messages:
1. `static_transform_publisher` publishes `world -> base_footprint` once
2. `robot_state_publisher` publishes all URDF transforms (including `base_footprint -> base_link` and children) once

Even though both are latched on `/tf_static`, RViz may query for transforms before it has received and processed both messages, leading to a temporary "unconnected trees" state. After a few seconds, the TF buffer should connect them, but RViz may need to be refreshed.

