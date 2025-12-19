# Tank Localization Configuration

## Point-LIO Configuration

Point-LIO configuration is managed in the point_lio package:
- **Config file**: `../point_lio/config/unilidar_l2.yaml`
- **Launch file**: This package wraps Point-LIO's launch file

## Future Configurations

This directory will contain:
- **`ekf_config.yaml`** - robot_localization EKF parameters (for sensor fusion)
- **`navsat_transform.yaml`** - GPS/GNSS transform parameters
- **`sensor_fusion.yaml`** - Multi-sensor fusion settings

## Current Setup

```
Tank Localization Stack:
├── Point-LIO (LiDAR odometry)
│   ├── Input: /lidar_front/cloud, /lidar_front/imu
│   └── Output: /Odometry, /cloud_registered, /path, TF: map→base_link
│
└── Future: robot_localization EKF
    ├── Fuses: Point-LIO odom + GNSS + IMU
    └── Output: Fused odometry + map→odom transform
```

## Usage

Launch Point-LIO via this integration package:
```bash
ros2 launch tank_localization point_lio.launch.py
```

With RViz:
```bash
ros2 launch tank_localization point_lio.launch.py rviz:=true
```

