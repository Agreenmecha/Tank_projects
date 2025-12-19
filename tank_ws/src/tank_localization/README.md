# Tank Localization Package

## Overview

Integration package for the tank's state estimation and localization system.

Currently wraps Point-LIO for LiDAR-based odometry. Future versions will include multi-sensor fusion (LiDAR + GNSS + IMU) using robot_localization.

## Current Features

- **Point-LIO Integration**: Launches Point-LIO with tank-specific configuration
- **Unitree L2 Front LiDAR**: Uses front-mounted L2 for odometry and mapping
- **TF Broadcasting**: Provides `map → base_link` transform

## Package Structure

```
tank_localization/
├── launch/
│   └── point_lio.launch.py          # Launches Point-LIO for tank
├── config/
│   └── README.md                     # Future configs (EKF, sensor fusion)
├── CMakeLists.txt
├── package.xml
└── README.md (this file)
```

## Dependencies

- **point_lio**: LiDAR odometry and mapping
- **tank_sensors**: Provides LiDAR and IMU data
- **tank_msgs**: Custom message types

## Usage

### Launch Point-LIO

```bash
# Basic launch (no RViz)
ros2 launch tank_localization point_lio.launch.py

# With visualization
ros2 launch tank_localization point_lio.launch.py rviz:=true
```

### Launch Full System

For complete tank operation, use the main bringup package:
```bash
ros2 launch tank_bringup sensors_localization.launch.py
```

## Input Topics

| Topic | Type | Source |
|-------|------|--------|
| `/lidar_front/cloud` | sensor_msgs/PointCloud2 | Unitree L2 front LiDAR |
| `/lidar_front/imu` | sensor_msgs/Imu | L2 built-in IMU |

## Output Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/Odometry` | nav_msgs/Odometry | LiDAR odometry (10 Hz) |
| `/cloud_registered` | sensor_msgs/PointCloud2 | Registered point cloud in map frame |
| `/path` | nav_msgs/Path | Trajectory path |

## TF Frames

Point-LIO broadcasts:
- **map → base_link**: Global localization transform

## Configuration

Point-LIO configuration is managed in:
- `../point_lio/config/unilidar_l2.yaml`

Key parameters:
- Frame IDs: `map`, `base_link`
- Publishing rate: 10 Hz
- LiDAR topic: `/lidar_front/cloud`
- IMU topic: `/lidar_front/imu`

## Future Enhancements

### Phase 2: Multi-Sensor Fusion
- Add `robot_localization` EKF for sensor fusion
- Fuse Point-LIO odometry with GNSS and IMU
- Improve accuracy and robustness

### Phase 3: Advanced Features
- Loop closure detection
- Map saving/loading
- Multiple LiDAR fusion (front + rear)

## Troubleshooting

### No odometry output
```bash
# Check if LiDAR is publishing
ros2 topic hz /lidar_front/cloud

# Check Point-LIO status
ros2 topic hz /Odometry
```

### Poor localization
- Ensure robot is stationary during IMU initialization
- Check Point-LIO logs for saturation warnings
- Verify LiDAR has sufficient features in environment

## Related Packages

- **point_lio**: Core LiDAR odometry algorithm
- **tank_sensors**: Sensor drivers and data processing
- **tank_bringup**: System-wide launch files
- **tank_navigation**: Navigation stack integration

## References

- [Point-LIO Paper](https://github.com/hku-mars/Point-LIO)
- [Point-LIO ROS2 Port](https://github.com/dfloreaa/point_lio_ros2)
- [Unitree L2 Documentation](https://github.com/unitreerobotics/unilidar_sdk2)

---

**Version**: 0.1.0  
**Status**: Point-LIO integration complete, sensor fusion planned  
**Last Updated**: December 2025

