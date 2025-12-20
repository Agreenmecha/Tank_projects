# Topic Connections - Full Autonomous System with Gamepad Override

Complete data flow from sensors → localization → navigation → motors, with manual override.

**Last Updated:** Dec 19, 2025

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        SENSOR LAYER                                 │
├─────────────────────────────────────────────────────────────────────┤
│  LiDAR Front      LiDAR Rear       GPS/GNSS         IMU             │
│  (/lidar_front)   (/lidar_rear)    (/gnss)          (in LiDAR)      │
└────────┬──────────────┬─────────────┬──────────────────┬────────────┘
         │              │             │                  │
         │ point clouds │             │ NavSatFix        │ IMU data
         │              │             │                  │
         ▼              ▼             ▼                  ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    LOCALIZATION LAYER                               │
├─────────────────────────────────────────────────────────────────────┤
│  Point-LIO                         navsat_transform                 │
│  (LiDAR-inertial odometry)         (GPS → map coords)               │
│                                                                      │
│  Publishes:                        Publishes:                       │
│  • /Odometry                       • /gps/filtered                  │
│  • TF: map → base_link             • TF: map → odom (correction)    │
└────────┬───────────────────────────────────────────────────────────┘
         │
         │ /Odometry, TFs
         │
         ▼
┌─────────────────────────────────────────────────────────────────────┐
│                     NAVIGATION LAYER                                │
├─────────────────────────────────────────────────────────────────────┤
│  Nav2 Stack:                                                        │
│  • Global Costmap (from both LiDARs)                                │
│  • Local Costmap (obstacle avoidance)                               │
│  • Path Planner (NavFn)                                             │
│  • Controller (DWB)                                                 │
│                                                                      │
│  Subscribes:                                                        │
│  • /Odometry (localization)                                         │
│  • /lidar_front/cloud, /lidar_rear/cloud (obstacles)                │
│  • TF tree (map → base_link)                                        │
│                                                                      │
│  Publishes:                                                         │
│  • /cmd_vel_nav (velocity commands) ────────────┐                   │
└─────────────────────────────────────────────────┼───────────────────┘
                                                  │
         ┌────────────────────────────────────────┘
         │
         │ /cmd_vel_nav (priority: 10)
         │
         ▼
┌─────────────────────────────────────────────────────────────────────┐
│                   COMMAND MUX LAYER (NEW!)                          │
├─────────────────────────────────────────────────────────────────────┤
│  Twist Mux (Priority-based command selector)                        │
│                                                                      │
│  Inputs (by priority):                                              │
│  1. /cmd_vel_safety   (priority: 200) ◄── Emergency stop            │
│  2. /cmd_vel_joy      (priority: 100) ◄── Remote gamepad (laptop)   │
│  3. /cmd_vel_nav      (priority: 10)  ◄── Nav2 autonomous           │
│                                                                      │
│  Output:                                                            │
│  • /cmd_vel ─────────────────────────────────────┐                  │
│                                                   │                  │
│  Status:                                          │                  │
│  • /twist_mux/selected (shows active input)       │                  │
└───────────────────────────────────────────────────┼──────────────────┘
                                                    │
         ┌──────────────────────────────────────────┘
         │
         │ /cmd_vel
         │
         ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      CONTROL LAYER                                  │
├─────────────────────────────────────────────────────────────────────┤
│  Safety Monitor          ODrive Interface                           │
│  (limits, E-stop)        (motor controller)                         │
│                                                                      │
│  Subscribes:             Subscribes:                                │
│  • /cmd_vel              • /cmd_vel                                 │
│  • /Odometry             • /odom (optional)                         │
│                                                                      │
│  Publishes:              Publishes:                                 │
│  • /cmd_vel_safe         • /odom (wheel odometry)                   │
│  • /estop_status         • /motor_status                            │
└────────┬────────────────────────┬───────────────────────────────────┘
         │                        │
         │ Motor commands         │ Encoder feedback
         │                        │
         ▼                        ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      HARDWARE LAYER                                 │
├─────────────────────────────────────────────────────────────────────┤
│  ODrive Motor Controllers                                           │
│  Left Track Motor (axis0)   Right Track Motor (axis1)               │
└─────────────────────────────────────────────────────────────────────┘
```

---

## New Gamepad Override Flow

```
┌────────────────────────────────────────────────────────┐
│                    LAPTOP                              │
│                                                        │
│  ┌──────────┐        ┌───────────────┐               │
│  │ Gamepad  │───────►│  joy_node     │               │
│  │ (Xbox)   │        │               │               │
│  └──────────┘        └───────┬───────┘               │
│                              │                        │
│                              ▼                        │
│                      ┌───────────────┐               │
│                      │ teleop_twist  │               │
│                      │     _joy      │               │
│                      └───────┬───────┘               │
│                              │                        │
│                              │ /cmd_vel_joy           │
│                              │ (over ROS2 network)    │
└──────────────────────────────┼────────────────────────┘
                               │
           Network (ROS_DOMAIN_ID=42)
                               │
┌──────────────────────────────┼────────────────────────┐
│                    JETSON    │                        │
│                              ▼                        │
│                      ┌────────────────┐              │
│   /cmd_vel_nav ─────►│   Twist Mux    │              │
│   (from Nav2)        │   (Priority)   │              │
│                      │                │              │
│                      │  gamepad: 100  │              │
│                      │  nav2: 10      │              │
│                      └───────┬────────┘              │
│                              │                        │
│                              ▼                        │
│                          /cmd_vel                     │
│                              │                        │
│                              ▼                        │
│                      ┌────────────────┐              │
│                      │ ODrive + Tank  │              │
│                      └────────────────┘              │
└────────────────────────────────────────────────────────┘
```

**Key:**
- Gamepad input has **priority 100** (high)
- Nav2 autonomous has **priority 10** (low)
- If no gamepad input for **500ms**, Nav2 takes over
- Hold **LB/L1 button** to enable gamepad control

---

## Topic List

### Sensors

| Topic | Type | Rate | Publisher | Description |
|-------|------|------|-----------|-------------|
| `/lidar_front/cloud` | `sensor_msgs/PointCloud2` | ~10 Hz | `unilidar_front_node` | Front LiDAR point cloud |
| `/lidar_front/imu` | `sensor_msgs/Imu` | ~100 Hz | `unilidar_front_node` | Front LiDAR IMU |
| `/lidar_rear/cloud` | `sensor_msgs/PointCloud2` | ~10 Hz | `unilidar_rear_node` | Rear LiDAR point cloud |
| `/lidar_rear/imu` | `sensor_msgs/Imu` | ~100 Hz | `unilidar_rear_node` | Rear LiDAR IMU |
| `/gnss/fix` | `sensor_msgs/NavSatFix` | ~5 Hz | `ublox_gps_node` | GPS position (lat/lon) |
| `/gnss/fix_velocity` | `geometry_msgs/TwistWithCovarianceStamped` | ~5 Hz | `ublox_gps_node` | GPS velocity |

### Localization

| Topic | Type | Rate | Publisher | Description |
|-------|------|------|-----------|-------------|
| `/Odometry` | `nav_msgs/Odometry` | ~10 Hz | `pointlio_mapping` | LiDAR-inertial odometry (map frame) |
| `/gps/filtered` | `sensor_msgs/NavSatFix` | ~5 Hz | `navsat_transform_node` | GPS with map alignment |
| `/odometry/gps` | `nav_msgs/Odometry` | ~5 Hz | `navsat_transform_node` | GPS in map coordinates |

### Navigation (Nav2)

| Topic | Type | Rate | Publisher | Description |
|-------|------|------|-----------|-------------|
| `/cmd_vel_nav` | `geometry_msgs/Twist` | ~20 Hz | `controller_server` | Nav2 velocity commands (via relay) |
| `/plan` | `nav_msgs/Path` | Variable | `planner_server` | Global path plan |
| `/local_plan` | `nav_msgs/Path` | ~10 Hz | `controller_server` | Local path being followed |
| `/goal_pose` | `geometry_msgs/PoseStamped` | On demand | User/GPS Manager | Goal position for Nav2 |
| `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | ~5 Hz | `local_costmap` | Local obstacle map |
| `/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | ~1 Hz | `global_costmap` | Global obstacle map |

### Command Multiplexer (NEW!)

| Topic | Type | Rate | Publisher | Description |
|-------|------|------|-----------|-------------|
| `/cmd_vel_joy` | `geometry_msgs/Twist` | ~20 Hz | `teleop_twist_joy` (laptop) | Gamepad commands (priority: 100) |
| `/cmd_vel_nav` | `geometry_msgs/Twist` | ~20 Hz | `controller_server` (Nav2) | Autonomous commands (priority: 10) |
| `/cmd_vel_safety` | `geometry_msgs/Twist` | On demand | Safety systems | Emergency stop (priority: 200) |
| `/cmd_vel` | `geometry_msgs/Twist` | ~20 Hz | `twist_mux` | **Final output to motors** |
| `/twist_mux/selected` | `std_msgs/String` | ~10 Hz | `twist_mux` | Active input source name |

### Motor Control

| Topic | Type | Rate | Publisher | Description |
|-------|------|------|-----------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | ~20 Hz | `twist_mux` | Velocity commands to ODrive |
| `/odom` | `nav_msgs/Odometry` | ~50 Hz | `odrive_interface_node` | Wheel odometry (odom frame) |
| `/motor_status` | Custom | ~1 Hz | `odrive_interface_node` | Motor health, current, temp |
| `/estop_status` | `std_msgs/Bool` | ~10 Hz | `safety_monitor_node` | Emergency stop state |

### GPS Waypoint Mission

| Topic | Type | Rate | Publisher | Description |
|-------|------|------|-----------|-------------|
| `/gps_waypoint_manager/command` | `std_msgs/String` | On demand | Web interface | Mission commands (JSON) |
| `/gps_waypoint_manager/status` | `std_msgs/String` | ~1 Hz | `gps_waypoint_manager` | Current mission status |

---

## TF Tree

```
map
 ├─ odom (published by: navsat_transform_node)
 │   └─ base_link (published by: Point-LIO)
 │       ├─ base_footprint
 │       ├─ lidar_front
 │       │   └─ lidar_front_scan
 │       ├─ lidar_rear
 │       │   └─ lidar_rear_scan
 │       ├─ gnss_link
 │       └─ camera_link (if enabled)
```

**Frame Purposes:**
- **`map`**: Global fixed frame (Point-LIO's world)
- **`odom`**: Local odometry frame (drift-corrected by GPS)
- **`base_link`**: Robot center
- **`lidar_front/rear`**: LiDAR sensor positions

---

## Verification Commands

### Check All Topics Publishing

```bash
# Sensors
ros2 topic hz /lidar_front/cloud        # Should be ~10 Hz
ros2 topic hz /lidar_rear/cloud         # Should be ~10 Hz
ros2 topic hz /gnss/fix                 # Should be ~5 Hz

# Localization
ros2 topic hz /Odometry                 # Should be ~10 Hz

# Navigation
ros2 topic hz /cmd_vel_nav              # Should be ~20 Hz (when Nav2 active)

# Gamepad (if connected)
ros2 topic hz /cmd_vel_joy              # Should be ~20 Hz (when gamepad active)

# Final output
ros2 topic hz /cmd_vel                  # Should be ~20 Hz

# Check active mux input
ros2 topic echo /twist_mux/selected
# Should show: "gamepad" or "navigation"

# Motors
ros2 topic hz /odom                     # Should be ~50 Hz
```

### Visualize Topic Graph

```bash
rqt_graph
```

**Look for:**
- `twist_mux` connected to `/cmd_vel`
- `controller_server` → `/cmd_vel_nav` → `twist_mux`
- `teleop_twist_joy` (laptop) → `/cmd_vel_joy` → `twist_mux`
- `twist_mux` → `/cmd_vel` → `odrive_interface_node`

### Check TF Tree

```bash
ros2 run tf2_tools view_frames
# Generates frames.pdf
```

---

## Integration Verification Checklist

### Sensors ✓
- [ ] Front LiDAR publishing point clouds
- [ ] Rear LiDAR publishing point clouds
- [ ] GPS publishing fix
- [ ] IMU data from LiDAR

### Localization ✓
- [ ] Point-LIO publishing `/Odometry`
- [ ] TF `map` → `base_link` available
- [ ] `navsat_transform_node` running (for GPS missions)

### Navigation ✓
- [ ] Nav2 stack running (all lifecycle nodes active)
- [ ] Costmaps populated from LiDAR data
- [ ] Can set goal in RViz
- [ ] `/cmd_vel_nav` published when navigating

### Gamepad Override (NEW!) ✓
- [ ] `twist_mux` node running
- [ ] Gamepad connected on laptop
- [ ] `/cmd_vel_joy` published when gamepad active
- [ ] `/twist_mux/selected` shows "gamepad" when active
- [ ] Nav2 resumes after gamepad timeout

### Motor Control ✓
- [ ] ODrive connected and calibrated
- [ ] `/cmd_vel` received by ODrive
- [ ] Robot responds to commands
- [ ] Safety monitor active

---

## Troubleshooting

### Gamepad Not Overriding Nav2

**Check:**
```bash
# Is gamepad publishing?
ros2 topic hz /cmd_vel_joy

# Is twist_mux seeing it?
ros2 topic echo /twist_mux/selected

# Is twist_mux running?
ros2 node list | grep twist_mux
```

**Fix:**
- Ensure `ROS_DOMAIN_ID=42` on both machines
- Check gamepad cable/wireless connection
- Verify twist_mux config: `cat ~/Tank_projects/tank_ws/src/tank_control/config/twist_mux.yaml`

### Nav2 Commands Not Reaching ODrive

**Check:**
```bash
# Is Nav2 publishing?
ros2 topic hz /cmd_vel_nav

# Is twist_mux forwarding?
ros2 topic hz /cmd_vel

# What's the mux selection?
ros2 topic echo /twist_mux/selected  # Should show "navigation"
```

**Fix:**
- Stop gamepad (to remove priority override)
- Check relay: `ros2 node list | grep relay`
- Verify Nav2 is navigating: `ros2 topic echo /plan`

### Robot Not Responding

**Check full chain:**
```bash
# 1. Nav2 output
ros2 topic echo /cmd_vel_nav

# 2. Mux output
ros2 topic echo /cmd_vel

# 3. ODrive receiving
ros2 node info /odrive_interface_node | grep Subscriptions
```

---

## Summary

### Data Flow (Gamepad Override Mode)

```
Sensors → Point-LIO → /Odometry → Nav2 → /cmd_vel_nav ──┐
                                                         │
                                                         ▼
                                                   ┌──────────┐
Laptop Gamepad → /cmd_vel_joy ────────────────────►│Twist Mux │
                                                   └────┬─────┘
                                                        │
                                                        ▼
                                                    /cmd_vel
                                                        │
                                                        ▼
                                              ODrive → Motors
```

**Priority:** Gamepad (100) > Nav2 (10)  
**Timeout:** 500ms after last gamepad input  
**Control:** Hold LB/L1 to enable gamepad

---

**Last Updated:** Dec 19, 2025  
**Status:** Fully integrated with gamepad override
