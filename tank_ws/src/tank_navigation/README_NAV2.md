# Tank Navigation with Nav2

## Overview

Complete autonomous navigation stack for the tracked tank using Nav2.

## What's Configured

### ✅ Complete Nav2 Stack
- **Controller**: DWB (Dynamic Window Approach) - tuned for tracked vehicle
- **Planner**: NavFn (A* algorithm) for global path planning
- **Costmaps**: Dual LiDAR integration (front + rear L2)
- **Behavior Trees**: Recovery behaviors (spin, backup, wait)
- **Velocity Smoother**: Gradual acceleration to prevent track slippage
- **Waypoint Follower**: Multi-waypoint missions

### ✅ Tracked Vehicle Optimizations
```yaml
Key Parameters (already configured):
- Max velocity: 0.5 m/s (adjust based on testing)
- Acceleration limit: 0.3 m/s² (prevents track slip)
- Robot radius: 0.5m (conservative)
- Goal tolerance: 0.25m / 14° (realistic for tracks)
- Costmap resolution: 0.05m (5cm)
```

### ✅ Dual LiDAR Integration
```yaml
Local Costmap (5m radius, real-time obstacles):
  - Front L2: /lidar_front/cloud_fixed
  - Rear L2: /lidar_rear/cloud_fixed
  - 360° obstacle detection
  - Update: 5 Hz, Publish: 2 Hz

Global Costmap (larger area, planning):
  - Same sensors, larger range
  - Update: 1 Hz
```

## File Structure

```
tank_navigation/
├── config/
│   └── nav2/
│       └── nav2_params.yaml       # Complete Nav2 configuration
├── launch/
│   ├── navigation.launch.py       # Nav2 only
│   ├── tank_nav_full.launch.py   # Complete system
│   └── gps_waypoint.launch.py    # Existing GPS waypoints
├── rviz/
│   └── nav2_default_view.rviz    # RViz config
└── README_NAV2.md (this file)
```

## Prerequisites

Before launching navigation:
1. ✅ Nav2 installed (`ros-humble-navigation2`)
2. ✅ LiDAR drivers configured
3. ✅ Point-LIO localization working
4. ✅ Robot URDF with correct TF tree

## Usage

### Option 1: Launch Everything Together (RECOMMENDED)

```bash
cd ~/Tank_projects/tank_ws
source install/setup.bash

# Full stack: LiDAR + Point-LIO + Nav2 + RViz
ros2 launch tank_navigation tank_nav_full.launch.py

# Without RViz (lighter)
ros2 launch tank_navigation tank_nav_full.launch.py rviz:=false
```

### Option 2: Launch Components Separately

```bash
# Terminal 1: LiDAR drivers
ros2 launch tank_sensors lidar_dual.launch.py

# Terminal 2: Point-LIO (localization)
ros2 launch tank_localization point_lio.launch.py

# Terminal 3: Nav2 (navigation)
ros2 launch tank_navigation navigation.launch.py

# Terminal 4: RViz (optional)
rviz2 -d ~/Tank_projects/tank_ws/src/tank_navigation/rviz/nav2_default_view.rviz
```

## First Test - Basic Navigation

### Step 1: Verify System Health

```bash
# Check all required topics are publishing
ros2 topic list | grep -E "Odometry|cloud_fixed|costmap"

# Should see:
# /Odometry (Point-LIO odometry)
# /lidar_front/cloud_fixed
# /lidar_rear/cloud_fixed
# /local_costmap/costmap_raw
# /global_costmap/costmap_raw

# Check TF tree
ros2 run tf2_tools view_frames
# Should show: map → base_link (from Point-LIO)
```

### Step 2: Set Navigation Goal in RViz

1. **Open RViz** (if not already)
2. **Add Nav2 Panel**: Panels → Add → Nav2 Panel
3. **Set 2D Pose Estimate**: Click toolbar button, click/drag on map
4. **Set Navigation Goal**: Click "Nav2 Goal" button, click destination on map
5. **Watch it go!**

### Step 3: Monitor Navigation

```bash
# Watch costmaps being updated
ros2 topic hz /local_costmap/costmap_raw

# Watch controller commands
ros2 topic echo /cmd_vel

# Check navigation status
ros2 topic echo /behavior_tree_log
```

## Common Issues & Fixes

### Issue: "No valid control trajectory"
**Cause**: Costmap has obstacles blocking all paths
**Fix**: 
- Check LiDAR topics are publishing
- Verify costmap is clearing properly
- May need to adjust `inflation_radius` in config

### Issue: Robot oscillates/doesn't reach goal
**Cause**: Controller gains need tuning
**Fix**: Edit `nav2_params.yaml`:
```yaml
controller_server:
  FollowPath:
    PathAlign.scale: 32.0  # Increase for tighter path following
    GoalDist.scale: 24.0   # Increase to prioritize reaching goal
```

### Issue: Robot turns too fast/slow
**Cause**: Velocity limits
**Fix**: Edit `nav2_params.yaml`:
```yaml
controller_server:
  FollowPath:
    max_vel_theta: 1.0     # Adjust rotation speed
    acc_lim_theta: 1.0     # Adjust rotation acceleration
```

### Issue: Robot gets stuck on obstacles
**Cause**: Robot footprint or inflation radius wrong
**Fix**: Edit `nav2_params.yaml`:
```yaml
local_costmap:
  local_costmap:
    robot_radius: 0.5      # Adjust to match actual robot size
    inflation_layer:
      inflation_radius: 0.55  # Should be > robot_radius
```

## Tuning for Your Tank

### Phase 1: Safety First (Current Settings)
```yaml
max_vel_x: 0.5 m/s          # Conservative
acc_lim_x: 0.3 m/s²         # Gentle
robot_radius: 0.5m          # Large safety margin
```

### Phase 2: After Testing
Test incrementally:
1. **Increase speed**: `max_vel_x: 0.8`
2. **Faster acceleration**: `acc_lim_x: 0.5`
3. **Tighter footprint**: `robot_radius: 0.4` (if safe)

### Phase 3: Terrain-Specific
```yaml
For slopes/rough terrain:
  - Decrease: max_vel_x, acc_lim_x
  - Increase: sim_time (look-ahead)
  
For smooth terrain:
  - Increase: max_vel_x, turning speed
  - Decrease: inflation_radius
```

## Waypoint Missions

### Define Waypoints (YAML format)

Create `tank_navigation/missions/test_mission.yaml`:
```yaml
waypoints:
  - pose:
      position: {x: 2.0, y: 0.0, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  - pose:
      position: {x: 2.0, y: 2.0, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}
  - pose:
      position: {x: 0.0, y: 2.0, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 1.0, w: 0.0}
```

### Send Waypoints Programmatically

```python
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

def main():
    rclpy.init()
    navigator = BasicNavigator()
    
    # Define waypoints
    waypoints = []
    for x, y in [(2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        waypoints.append(pose)
    
    # Follow waypoints
    navigator.followWaypoints(waypoints)
    
    # Wait until complete
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        print(f"Waypoint {feedback.current_waypoint + 1} of {len(waypoints)}")
    
    navigator.lifecycleShutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Safety Features

### Built-in (Already Configured)
- ✅ Obstacle detection (dual LiDAR, 360°)
- ✅ Collision avoidance (inflation layer)
- ✅ Recovery behaviors (backup, spin)
- ✅ Velocity limits (prevent track slip)
- ✅ Watchdog timers (node health monitoring)

### Recommended Additions
- ⏳ Slope detection (use IMU pitch/roll)
- ⏳ Emergency stop button
- ⏳ Geofence boundaries
- ⏳ Speed limits by terrain type

## Performance Monitoring

```bash
# CPU usage by Nav2 nodes
top -p $(pgrep -d',' -f nav2)

# Navigation metrics
ros2 topic hz /cmd_vel                    # Controller rate
ros2 topic hz /local_costmap/costmap_raw  # Costmap updates
ros2 topic echo /local_costmap/costmap    # View costmap data

# TF delays
ros2 run tf2_ros tf2_monitor
```

## Next Steps

1. **✅ Basic Test**: Navigate to simple goal in open area
2. **⏳ Tune Controller**: Adjust speeds based on tank performance  
3. **⏳ Test Obstacles**: Navigate around objects
4. **⏳ Waypoint Mission**: Follow multiple waypoints
5. **⏳ Complex Terrain**: Test on slopes, rough ground
6. **⏳ Add Safety**: Slope monitoring, emergency stop
7. **⏳ GPS Integration**: Fuse with GNSS for outdoor navigation

## Resources

- [Nav2 Docs](https://navigation.ros.org/)
- [Tuning Guide](https://navigation.ros.org/tuning/index.html)
- [DWB Controller](https://navigation.ros.org/configuration/packages/configuring-dwb-controller.html)

---

**Status**: ✅ Nav2 Configured and Ready to Test  
**Next**: Run first navigation test!

