# Tank Full Autonomy Integration Guide

This guide walks through launching and testing the complete autonomous navigation system.

---

## 🎯 What We're Integrating

```
┌─────────────────────────────────────────────────────────────┐
│                  FULL AUTONOMY STACK                        │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  [Robot URDF] → [Sensors] → [Point-LIO] → [Nav2] → [ODrive]│
│                                                             │
│  Components:                                                │
│    1. Robot Description (TF tree)                          │
│    2. Dual LiDAR + GPS sensors                             │
│    3. Point-LIO localization                               │
│    4. Nav2 navigation stack                                │
│    5. ODrive motor control                                 │
│    6. GPS waypoint system (optional)                       │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 🚀 Quick Start

### Basic Autonomous Navigation (No GPS Waypoints)

```bash
cd ~/Tank_projects/tank_ws
source install/setup.bash
ros2 launch tank_bringup full_autonomy.launch.py
```

This launches:
- ✅ Robot URDF + TF
- ✅ Dual LiDAR drivers
- ✅ GPS sensor (for future use)
- ✅ Point-LIO localization
- ✅ Nav2 navigation
- ✅ ODrive motor control

### With RViz Visualization

```bash
ros2 launch tank_bringup full_autonomy.launch.py rviz:=true
```

### GPS Waypoint Missions

```bash
ros2 launch tank_bringup full_autonomy.launch.py enable_gps_waypoints:=true
```

Then open the web interface:
```bash
firefox ~/Tank_projects/tank_ws/install/tank_navigation/share/tank_navigation/web/gps_waypoint_planner.html
```

### Testing Without Motors (Dry Run)

```bash
ros2 launch tank_bringup full_autonomy.launch.py enable_motors:=false
```

---

## 📋 Pre-Flight Checklist

Before launching the full system:

### Hardware
- [ ] ODrive powered on and connected via USB
- [ ] Both LiDAR units powered and connected
- [ ] GPS antenna has clear sky view
- [ ] Motors can move freely (no obstructions)
- [ ] Emergency stop button accessible

### Software
- [ ] Workspace built: `colcon build`
- [ ] Workspace sourced: `source install/setup.bash`
- [ ] ODrive tested independently (via teleop)
- [ ] LiDAR publishing data: `ros2 topic hz /lidar_front/cloud`

---

## 🔍 Step-by-Step Integration Test

### Step 1: Launch System

```bash
cd ~/Tank_projects/tank_ws
source install/setup.bash
ros2 launch tank_bringup full_autonomy.launch.py rviz:=true
```

**Expected output:**
```
[1/7] Starting Robot Description...
[2/7] Starting Sensors (LiDAR, GPS, Camera)...
[3/7] Starting Localization (Point-LIO)...
[4/7] Starting Navigation (Nav2)...
[5/7] Starting Motor Control (ODrive)...
[6/7] Starting GPS Waypoint System (if enabled)...
[7/7] Starting Visualization (RViz)...
Tank Autonomy System Ready!
```

### Step 2: Verify All Topics

In a new terminal:
```bash
source install/setup.bash
ros2 topic list
```

**Critical topics to check:**

| Topic | Expected Hz | Description |
|-------|-------------|-------------|
| `/lidar_front/cloud` | ~10 Hz | Front LiDAR |
| `/lidar_rear/cloud` | ~10 Hz | Rear LiDAR |
| `/Odometry` | ~10 Hz | Point-LIO odometry |
| `/cmd_vel` | ~0 Hz (idle) | Nav2 → ODrive commands |
| `/odrive/encoder_odom` | ~50 Hz | Motor encoders |
| `/gnss/fix` | ~5 Hz | GPS position |

Check topic rates:
```bash
ros2 topic hz /Odometry
ros2 topic hz /lidar_front/cloud
ros2 topic hz /odrive/encoder_odom
```

### Step 3: Verify TF Tree

```bash
ros2 run tf2_tools view_frames
# Open frames.pdf to see TF tree
```

**Expected TF structure:**
```
map
 └─ base_link (from Point-LIO)
     ├─ lidar_front
     ├─ lidar_rear
     ├─ gps_link
     └─ ... (other sensors)
```

### Step 4: Check Nav2 Status

```bash
# Check Nav2 lifecycle nodes
ros2 lifecycle nodes

# Expected nodes:
# - controller_server
# - planner_server
# - recoveries_server
# - bt_navigator
# - waypoint_follower
```

### Step 5: Visualize in RViz

In RViz, you should see:
- ✅ Robot model (URDF)
- ✅ Point clouds from both LiDARs
- ✅ Odometry pose (red arrow)
- ✅ Local costmap (obstacles nearby)
- ✅ Global costmap (full map)
- ✅ Nav2 tools (2D Goal Pose button)

### Step 6: Send a Simple Goal

**In RViz:**
1. Click "2D Goal Pose" button in toolbar
2. Click on the map to set a goal 2-3 meters away
3. Robot should start moving!

**Monitor:**
```bash
# Watch velocity commands
ros2 topic echo /cmd_vel

# Watch motor status
ros2 topic echo /odrive/motor_status
```

### Step 7: Emergency Stop Test

**Publish emergency stop:**
```bash
ros2 topic pub /emergency_stop std_msgs/Bool "{data: true}" --once
```

**Expected:** Robot should stop immediately.

**Release:**
```bash
ros2 topic pub /emergency_stop std_msgs/Bool "{data: false}" --once
```

---

## 🔧 Troubleshooting

### Problem: Some Topics Not Publishing

**Check:**
```bash
ros2 topic list
ros2 node list
```

**Common issues:**
- **LiDAR not found:** Check USB connections, power
- **GPS no fix:** Needs clear sky view, wait 2-3 minutes
- **ODrive not found:** Check USB cable, permissions

### Problem: Nav2 Won't Start

**Check lifecycle nodes:**
```bash
ros2 lifecycle nodes
ros2 lifecycle get /controller_server
```

**Manually configure/activate:**
```bash
ros2 lifecycle set /controller_server configure
ros2 lifecycle set /controller_server activate
```

### Problem: Robot Not Moving

**Check command flow:**
```bash
# 1. Is Nav2 publishing?
ros2 topic echo /cmd_vel

# 2. Is ODrive receiving?
ros2 node info /odrive_interface_node

# 3. Are motors in closed loop?
ros2 topic echo /odrive/motor_status
```

**Check for errors:**
```bash
ros2 topic echo /odrive/errors
ros2 topic echo /safety/warnings
```

### Problem: Robot Moving Erratically

**Check odometry:**
```bash
# Compare Point-LIO vs encoder odometry
ros2 topic echo /Odometry
ros2 topic echo /odrive/encoder_odom
```

**Tune Nav2 parameters:**
- Edit `tank_ws/src/tank_navigation/config/nav2/nav2_params.yaml`
- Adjust DWB controller velocities and accelerations
- Rebuild: `colcon build --packages-select tank_navigation`

### Problem: Collisions / Not Avoiding Obstacles

**Check costmaps:**
```bash
# View costmap in RViz
# Add display: Map → Topic: /local_costmap/costmap
```

**Check LiDAR data:**
```bash
ros2 topic hz /lidar_front/cloud
ros2 topic hz /lidar_rear/cloud
```

**Adjust costmap parameters:**
- Edit `tank_ws/src/tank_navigation/config/nav2/nav2_params.yaml`
- Increase `obstacle_range` and `raytrace_range`
- Adjust `inflation_radius`

---

## 📊 System Performance Monitoring

### Resource Usage

```bash
# CPU/Memory usage
htop

# ROS2 node statistics
ros2 node list
ros2 topic bw /lidar_front/cloud  # Bandwidth
```

### Data Flow Diagram

```bash
# Visualize complete ROS graph
rqt_graph
```

**Expected connections:**
```
unitree_lidar → Point-LIO → Nav2 → ODrive
        ↓          ↓         ↓        ↓
    costmap     /Odometry  /cmd_vel  motors
```

### Recording Data

```bash
# Record critical topics for analysis
ros2 bag record \
  /Odometry \
  /cmd_vel \
  /lidar_front/cloud \
  /odrive/encoder_odom \
  /gnss/fix \
  -o autonomous_test_$(date +%Y%m%d_%H%M%S)
```

---

## 🎯 Testing Autonomous Navigation

### Test 1: Short Distance Goal (2-3 meters)

```bash
# Launch system
ros2 launch tank_bringup full_autonomy.launch.py rviz:=true

# In RViz: Set 2D Goal Pose 2-3m away
# Watch robot navigate to goal
```

**Success criteria:**
- ✅ Robot plans path
- ✅ Robot moves smoothly
- ✅ Robot avoids obstacles
- ✅ Robot reaches goal within tolerance (2.5m for GPS config)

### Test 2: Around Obstacles

Place obstacles (boxes, cones) in path:
- Set goal behind obstacles
- Watch Nav2 plan around them
- Verify safety margins

### Test 3: Recovery Behaviors

Block robot's path after it starts:
- Robot should stop
- Wait for obstacle to clear
- Robot should replan and continue

### Test 4: Emergency Stop

While robot is moving:
```bash
ros2 topic pub /emergency_stop std_msgs/Bool "{data: true}" --once
```

**Expected:** Immediate stop, no coast

### Test 5: GPS Waypoint Mission

```bash
# Launch with GPS waypoints
ros2 launch tank_bringup full_autonomy.launch.py enable_gps_waypoints:=true

# Open web interface, place 3-4 waypoints
# Send mission
# Watch robot navigate to each waypoint
```

---

## ⚙️ Launch File Options

### All Available Arguments

```bash
ros2 launch tank_bringup full_autonomy.launch.py \
  use_sim_time:=false \              # Use real hardware time
  rviz:=true \                        # Launch RViz
  enable_gps:=true \                  # Enable GPS sensor
  enable_camera:=false \              # Enable camera (optional)
  enable_motors:=true \               # Enable motor control
  enable_gps_waypoints:=false \       # Enable GPS waypoint system
  nav2_params:=nav2_params_gps.yaml  # Use GPS-tuned params
```

### Common Configurations

**Field operation (no visualization):**
```bash
ros2 launch tank_bringup full_autonomy.launch.py rviz:=false
```

**GPS waypoint mission:**
```bash
ros2 launch tank_bringup full_autonomy.launch.py \
  enable_gps_waypoints:=true \
  nav2_params:=nav2_params_gps.yaml
```

**Testing without hardware:**
```bash
ros2 launch tank_bringup full_autonomy.launch.py \
  enable_motors:=false \
  enable_gps:=false \
  rviz:=true
```

---

## 📈 Next Steps After Integration

### Immediate:
1. ✅ Verify all topics connected
2. ✅ Test basic navigation (2-3m goals)
3. ✅ Test emergency stop
4. ✅ Tune Nav2 parameters for smooth motion

### Short-term:
5. Test GPS waypoint missions
6. Validate obstacle avoidance
7. Test recovery behaviors
8. Record and analyze performance data

### Long-term:
9. Add camera perception (terrain classification)
10. Implement advanced recovery behaviors
11. Add RTK GPS for centimeter accuracy
12. Mission planning interface

---

## 📚 Related Documentation

- **Nav2 Configuration:** `tank_ws/src/tank_navigation/README_NAV2.md`
- **GPS Waypoints:** `tank_ws/src/tank_navigation/README_GPS_WAYPOINTS.md`
- **Motor Control:** `tank_ws/src/tank_control/README.md`
- **Topic Connections:** `tank_ws/TOPIC_CONNECTIONS.md`
- **Point-LIO Setup:** `tank_ws/POINT_LIO_SUCCESS.md`

---

## 🆘 Emergency Procedures

### If Robot Goes Out of Control:

1. **Physical E-Stop:** Press hardware emergency stop button
2. **Software E-Stop:** 
   ```bash
   ros2 topic pub /emergency_stop std_msgs/Bool "{data: true}" --once
   ```
3. **Kill All Nodes:** `Ctrl+C` in launch terminal
4. **Power Off ODrive:** Disconnect power

### If System Crashes:

1. Check logs: `ros2 node info <node_name>`
2. Restart individual nodes if needed
3. Check topic connections: `rqt_graph`
4. Relaunch full system

---

## ✅ System Status

- ✅ **Robot Description:** Configured
- ✅ **Sensors:** LiDAR + GPS ready
- ✅ **Localization:** Point-LIO working
- ✅ **Navigation:** Nav2 configured
- ✅ **Motor Control:** ODrive tested with teleop
- ✅ **Integration:** Full launch file created
- ⏳ **Testing:** Ready for integration test

---

**Status:** Ready for first autonomous navigation test!  
**Next Step:** Run `ros2 launch tank_bringup full_autonomy.launch.py rviz:=true`

**Estimated Time to First Autonomous Drive:** 30-60 minutes (including testing and tuning)

---

**Last Updated:** Dec 19, 2025  
**Author:** Tank Autonomy Integration Team

