# Tank Navigation - Quick Reference Card

**Print this for field testing!**

---

## 🚨 Emergency Procedures

### Hardware E-Stop
**Physical button on robot** → Immediately cuts motor power

### Software Emergency Stop
```bash
# Send zero velocity
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0}, angular: {z: 0}}" --once

# Or kill navigation
ros2 lifecycle set /local_planner shutdown
```

### Pitch Angle Limits (IMU-based)
- **28°** = Warning (slow down)
- **32°** = Critical (crawl mode)
- **38°** = EMERGENCY STOP

Monitor: `ros2 topic echo /imu/data` (look at orientation)

---

## 📡 System Startup

### 1. Power On Sequence
1. Turn on main power
2. Wait for Jetson boot (~30s)
3. SSH into robot: `ssh tank@192.168.1.100`
4. Check sensor connections: `ls /dev/ttyUSB* /dev/video*`

### 2. Launch Navigation Stack
```bash
# Full system
ros2 launch tank_bringup tank_full.launch.py

# Or step-by-step for debugging:
# Terminal 1: Sensors
ros2 launch tank_sensors hardware.launch.py

# Terminal 2: Localization
ros2 launch tank_localization point_lio.launch.py

# Terminal 3: Perception
ros2 launch tank_perception lidar_processing.launch.py

# Terminal 4: Navigation
ros2 launch tank_navigation navigation.launch.py
```

### 3. Verify System Health
```bash
# Check all nodes running
ros2 node list

# Check critical topics
ros2 topic list | grep -E "(lidar|imu|odom|cmd_vel|gnss)"

# Check TF tree
ros2 run tf2_tools view_frames
```

---

## 🎮 Manual Control (Teleoperation)

```bash
# Keyboard teleop
ros2 launch tank_bringup teleop.launch.py

# Controls:
#   i/k = forward/backward
#   j/l = turn left/right
#   u/o = diagonal movement
#   space = stop
#   q/z = increase/decrease speed
```

### Safe Speeds for Testing
- **Flat ground:** 0.5-0.8 m/s
- **10° slopes:** 0.3-0.5 m/s
- **15° slopes:** 0.2-0.3 m/s (crawl)
- **>20° slopes:** <0.2 m/s (TEST INCREMENTALLY!)

---

## 🗺️ Autonomous Waypoint Navigation

### Set Single Waypoint
```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 10.0, y: 5.0, z: 0.0},
    orientation: {x: 0, y: 0, z: 0, w: 1}
  }
}" --once
```

### Mission with Multiple Waypoints
```bash
ros2 run tank_navigation waypoint_commander \
  --waypoints "[[10,0], [10,10], [0,10], [0,0]]" \
  --corridor-width 5.0
```

### Pause/Resume Mission
```bash
# Pause
ros2 service call /navigation/pause std_srvs/Trigger

# Resume
ros2 service call /navigation/resume std_srvs/Trigger

# Cancel
ros2 service call /navigation/cancel std_srvs/Trigger
```

---

## 📊 Real-Time Monitoring

### Critical Status Topics
```bash
# Safety status
ros2 topic echo /safety/status

# Pitch angle (degrees)
ros2 topic echo /safety/pitch

# Track slip %
ros2 topic echo /slip/status

# GNSS quality
ros2 topic echo /gnss/fix

# Motor temperature
ros2 topic echo /odrive/temperature
```

### Quick Health Check Script
```bash
# Run this to get overview
ros2 run tank_utils health_monitor.py

# Output:
# ✓ Point-LIO drift: 0.3% (OK)
# ✓ LiDAR packet loss: 0.1% (OK)
# ✓ Camera confidence: 0.82 (OK)
# ✓ GNSS HDOP: 1.2, Satellites: 12 (GOOD)
# ✓ Pitch: 8.2° (OK)
# ⚠ Motor temp: 72°C (WARNING - reduce speed if >80°C)
# ✓ Track slip: 3% (OK)
```

---

## 🔧 Parameter Tuning (Live)

### Speed Limits
```bash
# Reduce max speed to 1.0 m/s
ros2 param set /local_planner max_vel_x 1.0
```

### Slope Limits
```bash
# Make slope detection more conservative
ros2 param set /costmap_fusion slope_soft_limit 20.0
ros2 param set /costmap_fusion slope_hard_limit 25.0
```

### Inflation Radius
```bash
# Increase safety buffer
ros2 param set /local_planner inflation_radius 0.55
```

### Corridor Width (GNSS uncertainty)
```bash
# Widen corridor for poor GNSS
ros2 param set /global_planner corridor_width 6.0
```

---

## 📹 Data Recording

### Start Recording Session
```bash
# Automated (recommended)
./scripts/run_field_test.sh --test T2 --duration 30min

# Manual
ros2 bag record \
  /lidar_front/pointcloud \
  /lidar_rear/pointcloud \
  /camera/image_raw \
  /gnss/fix \
  /odom \
  /cmd_vel \
  /safety/status \
  -o ~/data/rosbags/manual_$(date +%Y%m%d_%H%M%S)
```

### Stop Recording
Press `Ctrl+C` in the terminal running `ros2 bag record`

---

## 🧪 Field Test Checklists

### Pre-Test (Every Session)
- [ ] Battery fully charged
- [ ] Hardware E-stop tested
- [ ] Sensor mounts secure (torque check)
- [ ] Camera lens clean
- [ ] LiDAR windows clean
- [ ] GNSS antenna unobstructed
- [ ] Geo-fence configured
- [ ] Test area clear of obstacles/people
- [ ] Teleoperate 50m to verify systems

### During Test
- [ ] Monitor pitch angle continuously
- [ ] Watch motor temperature
- [ ] Log GNSS fix quality
- [ ] Note terrain conditions (weather, lighting, slope)
- [ ] Record any anomalies (stuck events, weird behaviors)

### Post-Test
- [ ] Download rosbag
- [ ] Calculate KPIs (stuck/km, min clearance, speed distribution)
- [ ] Inspect tracks/motors for damage
- [ ] Check logs for errors/warnings
- [ ] Update parameter tuning notes

---

## 🐛 Common Issues & Fixes

### "Point-LIO diverging"
**Symptom:** Odom drifting rapidly, robot thinks it's moving when stationary
**Fix:** 
1. Check IMU calibration: robot should be level
2. Verify front L2 packet rate: `ros2 topic hz /lidar_front/pointcloud`
3. Restart Point-LIO: `ros2 lifecycle set /point_lio shutdown && ros2 lifecycle set /point_lio configure`

### "Camera segmentation slow"
**Symptom:** TensorRT inference >50ms, perception lagging
**Fix:**
1. Check GPU utilization: `tegrastats` on Jetson
2. Verify TensorRT engine loaded (not ONNX fallback)
3. Reduce resolution: switch from 1280x720 to 960x600

### "GNSS poor fix"
**Symptom:** HDOP >5, <6 satellites, jumping position
**Fix:**
1. Check antenna placement (obstructed?)
2. Widen corridor: `ros2 param set /global_planner corridor_width 7.0`
3. Rely more on Point-LIO: navigate shorter missions (<500m)

### "Tracks slipping on slope"
**Symptom:** Encoder shows movement, Point-LIO shows <50% actual velocity
**Fix:**
1. Reduce speed: `ros2 param set /local_planner max_vel_x 0.4`
2. Approach slope at angle (not straight uphill)
3. If slip >30%, abort - insufficient traction

### "Robot refuses to reverse"
**Symptom:** Stuck facing obstacle, won't trigger reverse mode
**Fix:**
1. Check rear LiDAR data: `ros2 topic echo /lidar_rear/pointcloud | head -20`
2. Verify reverse safety gate: `ros2 topic echo /reverse/gate_status`
3. Manually trigger: `ros2 service call /enable_reverse std_srvs/Trigger`

### "Pitch warning at <20°"
**Symptom:** False pitch warnings on moderate slopes
**Fix:**
1. Calibrate IMU: ensure level at startup
2. Check for vibration (motors causing false readings)
3. Increase warning threshold: `ros2 param set /safety_monitor pitch_warning 30.0` (ONLY if validated safe!)

---

## 📞 Contact During Field Tests

**Developer:** [Your phone/email]

**Emergency Protocol:**
1. Hardware E-stop
2. Power off robot
3. Call for assistance if robot inaccessible or dangerous

---

## 🎯 Performance Targets (MVP)

| Metric | Target | Current |
|--------|--------|---------|
| Stuck events/km | ≤0.2 | ___ |
| Collisions | 0 | ___ |
| Tip-over events | 0 | ___ |
| Min obstacle clearance | >0.3m | ___ |
| Point-LIO drift | <1% | ___ |
| Max validated slope | ≥20° | ___ |
| Track slip events | <5/km | ___ |
| GNSS fix rate | >80% | ___ |

---

**Last Updated:** 2025-01-31
**Plan Version:** v1.0 (30° slope, GNSS-only, Isaac ROS)

