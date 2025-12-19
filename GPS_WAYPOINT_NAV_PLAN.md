# GPS Waypoint Navigation Implementation Plan

## 🎯 Goal
Navigate the tank to GPS waypoints using global positioning, with LiDAR for local obstacle avoidance.

## 📋 System Architecture

```
GPS/GNSS (ZED-F9P) ────────────────────┐
                                       │
IMU (LiDAR or standalone) ─────────────┤
                                       │
                                       ├──> Waypoint Follower ──> /cmd_vel ──> ODrive
                                       │
LiDAR (front/rear) ────> Costmap ──────┘
                         (obstacle avoidance)
```

**Why No Wheel Odometry:**
- Tank tracks slip on various surfaces (mud, grass, loose terrain)
- Encoder data unreliable for absolute positioning
- GPS provides ground truth position
- IMU provides orientation

## 🔧 Components to Build

### Phase 1: GPS + IMU Setup (No Sensor Fusion Needed)
**Simple and Robust for Tank Tracks**

**Inputs:**
- `/gnss/fix` - GPS position (sensor_msgs/NavSatFix) from ZED-F9P
  - Provides: latitude, longitude, altitude
  - RTK capable for cm-level accuracy
  
- `/imu/data` (optional) - Orientation from LiDAR IMU or magnetometer
  - Provides: heading (yaw)
  - Fallback: Calculate heading from GPS movement

**Output:**
- Direct GPS coordinates used for navigation
- No sensor fusion required (simpler, more reliable)

---

### Phase 2: Waypoint Follower
**Package:** `tank_navigation` (create new)

**Node:** `gps_waypoint_follower_node.py`

**Functionality:**
- Accept GPS waypoint goals (lat/lon)
- Convert to local coordinates
- Calculate heading and distance
- Publish velocity commands
- Handle waypoint sequences

**Interface:**
- Service: `/set_gps_waypoint` (lat, lon, tolerance)
- Action: `/navigate_to_waypoint` (for mission sequences)
- Topic: `/cmd_vel` (output to ODrive)

---

### Phase 3: Obstacle Avoidance
**Package:** `nav2_costmap_2d` (or custom)

**Functionality:**
- Build local costmap from LiDAR
- Dynamic window approach (DWA) or TEB planner
- Override velocity commands if obstacle detected

**Integration:**
- Subscribe to `/lidar_front/cloud` and `/lidar_rear/cloud`
- Modify `/cmd_vel` before ODrive receives it
- Emergency stop if path blocked

---

## 🚀 Implementation Steps

### Step 1: GPS Waypoint Follower ✅ (Next)
**Simple GPS-only navigation (no sensor fusion needed)**

1. Create: `tank_navigation/tank_navigation/gps_waypoint_follower.py`
2. Implement:
   - Subscribe to `/gnss/fix` (GPS position)
   - Subscribe to `/imu/data` (orientation) or calculate from GPS
   - Calculate distance and bearing to waypoint
   - Publish velocity commands to `/cmd_vel`
3. Add services:
   - `/set_waypoint` - Set target GPS coordinate
   - `/cancel_waypoint` - Stop navigation
4. Test: Navigate to single GPS waypoint

### Step 2: Create Waypoint Follower
1. Create package: `tank_navigation`
2. Implement: `gps_waypoint_follower_node.py`
3. Add: Pure pursuit or PID controller
4. Test: Navigate to single waypoint

### Step 3: Add Obstacle Avoidance
1. Setup costmap from LiDAR
2. Implement local planner
3. Integrate with waypoint follower
4. Test: Navigate around obstacles

### Step 4: Mission Planner (Future)
1. Waypoint sequences
2. Mission state machine
3. Failsafe behaviors

---

## 📊 Current Hardware Status

### Sensors Available:
- ✅ **GPS:** ZED-F9P GNSS (RTK capable)
  - Topic: `/gnss/fix` (NavSatFix)
  - 5 Hz update rate
  - RTK: cm-level accuracy with NTRIP corrections
  - Standard: ~2m accuracy

- ⚠️ **IMU:** Available from LiDAR
  - Topics: `/lidar_front/imu`, `/lidar_rear/imu`
  - Provides orientation (roll, pitch, yaw)
  - Alternative: Calculate heading from GPS movement

- ✅ **LiDAR:** Dual Unitree L2 (front + rear)
  - Topics: `/lidar_front/cloud`, `/lidar_rear/cloud`
  - For obstacle detection only (not localization)

- ❌ **Wheel Odometry:** NOT USED
  - Tank tracks slip on terrain
  - Unreliable for absolute positioning
  - Only used for smooth motor control

### Control Stack:
- ✅ **Motor Control:** ODrive interface working
- ✅ **Differential Drive:** Kinematics implemented
- ✅ **Direction Transitions:** Smooth direction changes

---

## 🎓 Key Algorithms

### 1. GPS Waypoint Navigation (Our Approach)
**Simple "point and drive" for GPS-only:**

1. **Calculate bearing to waypoint:**
   ```python
   # Haversine formula for bearing
   bearing = atan2(sin(Δlon) * cos(lat2), 
                   cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(Δlon))
   ```

2. **Calculate distance to waypoint:**
   ```python
   # Haversine formula for distance
   distance = 2 * R * asin(sqrt(sin²(Δlat/2) + cos(lat1)*cos(lat2)*sin²(Δlon/2)))
   ```

3. **Control strategy:**
   - If distance > threshold: Rotate to face waypoint, then drive forward
   - If obstacle detected: Stop, avoid, recalculate
   - If distance < tolerance: Waypoint reached

### 2. Heading Control
**Two options:**
- **Option A (with IMU):** Compare IMU heading with target bearing, PID control
- **Option B (GPS-only):** Calculate heading from GPS position changes (requires movement)

### 3. Obstacle Avoidance
- Monitor LiDAR for obstacles in path
- Stop if obstacle < safe distance
- Simple reactive behavior (stop and wait, or simple avoidance)

---

## 📝 Configuration Parameters

### Waypoint Following:
- `waypoint_tolerance`: 2.0 m (how close to waypoint)
- `max_linear_velocity`: 1.5 m/s
- `max_angular_velocity`: 2.0 rad/s
- `lookahead_distance`: 3.0 m (pure pursuit)
- `heading_threshold`: 0.2 rad (align before moving)

### GPS:
- `gps_update_rate`: 5 Hz (from ZED-F9P config)
- `gps_accuracy_threshold`: 5.0 m (minimum fix quality to navigate)
- `rtk_fix_required`: false (use standard GPS if RTK unavailable)

### Safety:
- `obstacle_distance`: 1.5 m (stop distance)
- `emergency_stop_distance`: 0.5 m

---

## ✅ Success Criteria

**Phase 1 Complete:**
- [ ] GPS position received and validated
- [ ] Heading calculated (from IMU or GPS movement)
- [ ] Drive to single GPS waypoint
- [ ] Stop within tolerance

**Phase 2 Complete:**
- [ ] Navigate to single GPS waypoint
- [ ] Stop within tolerance
- [ ] Handle waypoint sequences

**Phase 3 Complete:**
- [ ] Detect obstacles with LiDAR
- [ ] Navigate around obstacles
- [ ] Reach waypoint avoiding obstacles

---

## 🛠️ Next Immediate Actions

1. ✅ Create `gps_waypoint_follower_node.py` in `tank_navigation`
2. ✅ Implement GPS position parsing and coordinate transforms
3. ✅ Implement bearing/distance calculations (Haversine)
4. ✅ Add heading control (PID for rotation)
5. ✅ Add forward drive control
6. ✅ Create service interface for setting waypoints
7. ✅ Test: Drive to first GPS waypoint!

**Advantages of GPS-Only Approach:**
- ✅ Simpler (no sensor fusion complexity)
- ✅ More robust (no slip errors accumulating)
- ✅ Works on any terrain (mud, grass, rocks)
- ✅ Absolute positioning (no drift)
- ✅ Perfect for outdoor tank navigation

**Ready to start coding!** 🌍

