# Tank Autonomous Navigation - Topic Connections

This document maps all ROS2 topics, their publishers, subscribers, and data flow through the autonomous navigation system.

## System Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                     AUTONOMOUS NAVIGATION STACK                 │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐│
│  │ Sensors  │───▶│Localiz.  │───▶│  Nav2    │───▶│  Motors  ││
│  │(LiDAR+GPS)│    │Point-LIO │    │  Stack   │    │  ODrive  ││
│  └──────────┘    └──────────┘    └──────────┘    └──────────┘│
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## 1. Sensor Topics

### LiDAR (Unitree L2 Dual Setup)

| Topic | Type | Publisher | Subscriber | Hz | Description |
|-------|------|-----------|------------|----|----|
| `/lidar_front/cloud` | `sensor_msgs/PointCloud2` | `unitree_lidar_ros2` | Point-LIO, Nav2 Costmap | ~10 | Front LiDAR point cloud |
| `/lidar_front/imu` | `sensor_msgs/Imu` | `unitree_lidar_ros2` | Point-LIO, Safety Monitor | ~100 | Front LiDAR IMU |
| `/lidar_rear/cloud` | `sensor_msgs/PointCloud2` | `unitree_lidar_ros2` | Nav2 Costmap | ~10 | Rear LiDAR point cloud |
| `/lidar_rear/imu` | `sensor_msgs/Imu` | `unitree_lidar_ros2` | Safety Monitor | ~100 | Rear LiDAR IMU |

### GPS (u-blox ZED-F9P)

| Topic | Type | Publisher | Subscriber | Hz | Description |
|-------|------|-----------|------------|----|----|
| `/gnss/fix` | `sensor_msgs/NavSatFix` | `ublox_dgnss_node` | navsat_transform | ~5 | GPS position (lat/lon/alt) |
| `/gnss/navpvt` | `ublox_ubx_msgs/UBX_NAV_PVT` | `ublox_dgnss_node` | GPS Waypoint Manager | ~5 | Full navigation solution |
| `/ubx_nav_hp_pos_llh` | `ublox_ubx_msgs/UBX_NAV_HPPOSLLH` | `ublox_nav_sat_fix_hp_node` | navsat_transform | ~5 | High-precision GPS |

### Camera (e-CAM25 - Optional)

| Topic | Type | Publisher | Subscriber | Hz | Description |
|-------|------|-----------|------------|----|----|
| `/camera/image_raw` | `sensor_msgs/Image` | `argus_camera_node` | (future perception) | ~30 | Raw camera image |

---

## 2. Localization Topics

### Point-LIO Outputs

| Topic | Type | Publisher | Subscriber | Hz | Description |
|-------|------|-----------|------------|----|----|
| `/Odometry` | `nav_msgs/Odometry` | `laserMapping` | Nav2, Safety Monitor | ~10 | LiDAR-Inertial odometry |
| `/point_lio/cloud_registered` | `sensor_msgs/PointCloud2` | `laserMapping` | RViz | ~10 | Registered point cloud (map frame) |
| `/point_lio/path` | `nav_msgs/Path` | `laserMapping` | RViz | ~1 | Robot trajectory history |

### TF Frames (Published by Point-LIO)

| Frame | Parent | Publisher | Description |
|-------|--------|-----------|-------------|
| `map` | - | `laserMapping` | Global fixed frame |
| `base_link` | `map` | `laserMapping` | Robot base frame |

---

## 3. Navigation Topics (Nav2)

### Nav2 Inputs

| Topic | Type | Publisher | Subscriber | Hz | Description |
|-------|------|-----------|------------|----|----|
| `/Odometry` | `nav_msgs/Odometry` | Point-LIO | Nav2 Controller | ~10 | Robot odometry |
| `/lidar_front/cloud` | `sensor_msgs/PointCloud2` | LiDAR Driver | Nav2 Costmap | ~10 | Front obstacles |
| `/lidar_rear/cloud` | `sensor_msgs/PointCloud2` | LiDAR Driver | Nav2 Costmap | ~10 | Rear obstacles |

### Nav2 Outputs

| Topic | Type | Publisher | Subscriber | Hz | Description |
|-------|------|-----------|------------|----|----|
| `/cmd_vel` | `geometry_msgs/Twist` | `controller_server` | ODrive Interface | ~20 | **Velocity commands to motors** |
| `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | `controller_server` | RViz | ~5 | Local obstacle map |
| `/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | `planner_server` | RViz | ~1 | Global obstacle map |
| `/plan` | `nav_msgs/Path` | `planner_server` | RViz, Controller | ~1 | Global path plan |

### Nav2 Goal Input

| Topic | Type | Publisher | Subscriber | Description |
|-------|------|-----------|------------|-------------|
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz, GPS Manager | Nav2 BT Navigator | Single navigation goal |
| `/waypoints` | `nav_msgs/Path` | GPS Manager | Nav2 Waypoint Follower | Multi-waypoint mission |

---

## 4. Motor Control Topics

### ODrive Interface

| Topic | Type | Publisher | Subscriber | Hz | Description |
|-------|------|-----------|------------|----|----|
| `/cmd_vel` | `geometry_msgs/Twist` | **Nav2 or Teleop** | ODrive Interface | ~20 | **Motor velocity commands** |
| `/odrive/encoder_odom` | `nav_msgs/Odometry` | ODrive Interface | (future: EKF fusion) | ~50 | Wheel encoder odometry |
| `/odrive/motor_status` | `sensor_msgs/JointState` | ODrive Interface | Safety Monitor | ~20 | Motor currents, velocities |
| `/odrive/errors` | `std_msgs/Bool` | ODrive Interface | Safety Monitor | ~20 | ODrive error status |
| `/emergency_stop` | `std_msgs/Bool` | Safety Monitor, E-Stop Button | ODrive Interface | event | Emergency stop trigger |

---

## 5. GPS Waypoint Navigation Topics

### GPS Coordinate Conversion

| Topic | Type | Publisher | Subscriber | Hz | Description |
|-------|------|-----------|------------|----|----|
| `/gnss/fix` | `sensor_msgs/NavSatFix` | ublox_dgnss | navsat_transform | ~5 | GPS position |
| `/gps/filtered` | `sensor_msgs/NavSatFix` | navsat_transform | GPS Manager | ~5 | Filtered GPS in map frame |
| `/gps/odom` | `nav_msgs/Odometry` | navsat_transform | GPS Manager | ~5 | GPS as odometry (map frame) |

### GPS Waypoint Input (Web Interface)

| Topic | Type | Publisher | Subscriber | Hz | Description |
|-------|------|-----------|------------|----|----|
| `/gps_waypoints` | `geographic_msgs/GeoPath` | ROSBridge (web) | GPS Manager | event | GPS waypoint list from map |

---

## 6. Safety & Monitoring Topics

| Topic | Type | Publisher | Subscriber | Hz | Description |
|-------|------|-----------|------------|----|----|
| `/safety/status` | `std_msgs/String` | Safety Monitor | RViz, Logger | ~10 | Safety system status |
| `/safety/warnings` | `std_msgs/String` | Safety Monitor | Logger | event | Warning messages |
| `/emergency_stop` | `std_msgs/Bool` | Safety Monitor | ODrive Interface | event | E-stop trigger |

---

## Critical Data Flow

### For Autonomous Navigation:

```
1. LiDAR → Point-LIO → /Odometry → Nav2
2. LiDAR → Nav2 Costmaps (obstacles)
3. Nav2 → /cmd_vel → ODrive → Motors
4. Motors → /odrive/encoder_odom → (future: sensor fusion)
```

### For GPS Waypoint Missions:

```
1. GPS → navsat_transform → map coordinates
2. Web Interface → ROSBridge → /gps_waypoints → GPS Manager
3. GPS Manager → /goal_pose or /waypoints → Nav2
4. Nav2 → /cmd_vel → ODrive → Motors
```

---

## Topic Remappings

### In Point-LIO Launch:
- Point-LIO's `/aft_mapped_to_init` **→** `/Odometry`

### In GPS Waypoint Launch:
- ublox's `/fix` **→** `/gnss/fix`
- ublox's high-precision **→** `/ubx_nav_hp_pos_llh`

### In Nav2:
- **No remapping needed** - Nav2 publishes directly to `/cmd_vel`
- ODrive subscribes directly to `/cmd_vel`

---

## Verification Commands

### Check all topics are publishing:
```bash
ros2 topic list
ros2 topic hz /cmd_vel
ros2 topic hz /Odometry
ros2 topic hz /lidar_front/cloud
ros2 topic hz /gnss/fix
```

### Check Nav2 → ODrive connection:
```bash
# Should show Nav2 publishing, ODrive subscribing
ros2 topic info /cmd_vel
ros2 topic echo /cmd_vel
```

### Check Point-LIO → Nav2 connection:
```bash
ros2 topic info /Odometry
ros2 topic echo /Odometry --once
```

### Visualize complete topic graph:
```bash
ros2 run rqt_graph rqt_graph
```

---

## Status

- ✅ **Sensor Topics**: Configured and tested
- ✅ **Localization Topics**: Working (Point-LIO)
- ✅ **Motor Control Topics**: Tested with teleop
- ✅ **Nav2 Topics**: Configured (needs integration test)
- ✅ **GPS Topics**: Configured (needs hardware test)
- ⏳ **Full Integration**: Ready for testing

---

**Last Updated**: Dec 19, 2025  
**Next Step**: Launch `full_autonomy.launch.py` and verify all topic connections with `rqt_graph`

