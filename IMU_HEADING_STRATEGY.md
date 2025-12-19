# IMU Heading Strategy for Tank Navigation

## 🧭 Hardware: Dual 6DOF IMUs (No Magnetometer)

**Unitree L2 LiDAR IMUs:**
- ✅ 3-axis Gyroscope (angular velocity)
- ✅ 3-axis Accelerometer (linear acceleration)
- ❌ NO Magnetometer (no compass, no absolute north)

**Implications:**
- Can measure orientation changes accurately
- Can integrate gyro to get yaw angle
- **Yaw will drift over time** without absolute reference
- Roll/pitch stable from accelerometer gravity vector
- Need external reference to correct yaw drift

---

## 📊 Heading Accuracy Timeline

```
Time        IMU Yaw Error    Notes
─────────────────────────────────────────────────────
0 sec       0°               Initial heading (unknown absolute)
1 min       ±1-2°            Very accurate
5 min       ±5-10°           Starting to drift
30 min      ±20-50°          Significant drift
Hours       ±90°+            Lost orientation
```

**For GPS waypoint navigation:**
- ✅ Missions under 5 minutes: IMU alone is fine
- ⚠️ Longer missions: Need drift correction

---

## 🔄 Drift Correction Strategy

### **Complementary Filter: IMU + GPS**

```python
# High-frequency IMU for instant response
imu_heading = integrate_gyro()  # 100+ Hz

# Low-frequency GPS for absolute reference
if rover_is_moving() and gps_fix_good():
    gps_heading = calculate_gps_heading()  # 5 Hz
    
    # Slowly correct IMU drift (complementary filter)
    heading_error = normalize_angle(gps_heading - imu_heading)
    
    if abs(heading_error) > DRIFT_THRESHOLD:  # e.g., 10 degrees
        # Apply small correction to prevent jumps
        correction = heading_error * CORRECTION_GAIN  # e.g., 0.05
        imu_heading_offset += correction
```

### **When to Correct:**
- ✅ Rover moving > 0.5 m/s (GPS heading valid)
- ✅ GPS fix quality good (DOP < threshold)
- ✅ Straight-line motion (not turning)
- ❌ Stationary (GPS heading invalid)
- ❌ Turning (GPS heading lags)

---

## 🎯 Implementation Approach

### **Phase 1: IMU Fusion Node**

**Purpose:** Combine dual 6DOF IMUs for better accuracy

```python
class IMUFusionNode:
    def __init__(self):
        # Subscribers
        self.sub_front = self.subscribe('/lidar_front/imu')
        self.sub_rear = self.subscribe('/lidar_rear/imu')
        
        # Publisher
        self.pub_fused = self.publish('/imu/fused')
        
        # State
        self.yaw_offset = 0.0  # For GPS drift correction
    
    def fuse_imus(self, imu_front, imu_rear):
        # Average angular velocities (gyro)
        avg_angular_vel = (imu_front.angular_velocity + 
                          imu_rear.angular_velocity) / 2
        
        # Average linear accelerations (accel)
        avg_linear_accel = (imu_front.linear_acceleration + 
                           imu_rear.linear_acceleration) / 2
        
        # Average orientations (SLERP for quaternions)
        avg_orientation = slerp(imu_front.orientation, 
                                imu_rear.orientation, 0.5)
        
        # Apply drift correction offset
        avg_orientation = apply_yaw_offset(avg_orientation, 
                                           self.yaw_offset)
        
        return fused_imu
    
    def correct_yaw_drift(self, gps_heading, rover_velocity):
        """Called by GPS waypoint follower when moving"""
        if rover_velocity < 0.5:
            return  # GPS heading unreliable
        
        current_yaw = self.get_current_yaw()
        heading_error = normalize_angle(gps_heading - current_yaw)
        
        # Slowly correct drift (complementary filter)
        if abs(heading_error) > 10.0:  # degrees
            correction = heading_error * 0.05  # 5% correction per update
            self.yaw_offset += correction
```

---

### **Phase 2: GPS Waypoint Follower**

**Heading Usage:**

```python
class GPSWaypointFollower:
    def navigate_to_waypoint(self, target_lat, target_lon):
        # Get current state
        current_position = self.gps_fix  # From /gnss/fix
        current_heading = self.imu_yaw   # From /imu/fused
        
        # Calculate target bearing
        target_bearing = calculate_bearing(current_position, 
                                          target_lat, target_lon)
        
        # Calculate heading error
        heading_error = normalize_angle(target_bearing - current_heading)
        
        # Control strategy
        if abs(heading_error) > ALIGNMENT_THRESHOLD:  # e.g., 10°
            # Rotate in place
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = pid_heading.update(heading_error)
        else:
            # Drive forward (aligned)
            cmd_vel.linear.x = cruise_speed
            cmd_vel.angular.z = 0.0
            
            # Drift correction (when moving straight)
            if self.velocity > 0.5:
                gps_heading = self.calculate_gps_heading()
                self.imu_fusion.correct_yaw_drift(gps_heading, 
                                                  self.velocity)
```

---

## 🧪 Testing Strategy

### **Test 1: IMU Fusion Validation**
1. Park rover, let IMUs stabilize
2. Rotate rover 360° manually
3. Check if fused IMU returns to start heading
4. **Expected:** Small error (±5°) due to integration

### **Test 2: Short Mission (No Drift)**
1. Set waypoint 50m away
2. Navigate using IMU only
3. **Expected:** Accurate (mission < 1 min)

### **Test 3: Drift Detection**
1. Park rover for 10 minutes
2. Monitor IMU yaw drift
3. **Expected:** Drift visible after 5-10 min

### **Test 4: GPS Drift Correction**
1. Set waypoint 200m away (>2 min mission)
2. Enable GPS drift correction
3. **Expected:** Accurate arrival despite IMU drift

---

## 📝 Configuration Parameters

```yaml
imu_fusion_node:
  ros__parameters:
    # Fusion
    fusion_rate: 100.0  # Hz - IMU update rate
    outlier_threshold: 2.0  # Reject IMU readings with large disagreement
    
    # Drift correction
    drift_correction_enabled: true
    drift_threshold: 10.0  # degrees - correct if error > this
    correction_gain: 0.05  # Slow correction to prevent jumps
    min_velocity_for_correction: 0.5  # m/s - GPS heading valid
    
    # Quality checks
    max_angular_velocity: 5.0  # rad/s - reject if spinning too fast
    min_gps_quality: 5  # Minimum satellites for correction
```

---

## ✅ Advantages of This Approach

**IMU (Short-term):**
- ✅ High frequency (100+ Hz)
- ✅ Works when stationary
- ✅ Smooth, responsive
- ✅ Low latency

**GPS Correction (Long-term):**
- ✅ Prevents unbounded drift
- ✅ Provides absolute reference
- ✅ Simple complementary filter
- ✅ No complex sensor fusion needed

**Result:**
- ✅ Best of both worlds
- ✅ Accurate heading at all times
- ✅ Simple to implement
- ✅ Robust for outdoor navigation

---

## 🚨 Limitations to Accept

1. **Initial Heading Unknown**
   - IMU starts with arbitrary heading
   - First GPS heading (when moving) sets absolute reference
   - **Workaround:** Start mission by driving forward 5m to initialize

2. **Stationary Drift**
   - Long stops allow drift accumulation
   - **Workaround:** Brief movement every 5 min to correct

3. **Indoor/GPS-denied**
   - No drift correction without GPS
   - **Workaround:** Rely on IMU alone (acceptable for short missions)

4. **Rapid Maneuvers**
   - GPS heading lags during turns
   - **Workaround:** Only correct during straight-line motion

---

**This strategy is perfect for outdoor GPS waypoint navigation!** 🎯

