# Point-LIO Quick Start Guide

## ✅ Installation Complete!

Point-LIO ROS2 has been successfully installed and verified on your system.

## Quick Start (Unitree L2 LiDAR)

### 1. Launch Point-LIO

```bash
cd ~/Tank_projects/tank_ws
source install/setup.bash
ros2 launch point_lio mapping_unilidar_l2.launch.py
```

**Without RViz (faster)**:
```bash
ros2 launch point_lio mapping_unilidar_l2.launch.py rviz:=false
```

### 2. Launch with LiDAR Driver

If you need to start the L2 LiDAR driver:

**Terminal 1 - LiDAR Driver**:
```bash
cd ~/Tank_projects/tank_ws
source install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py
```

**Terminal 2 - Point-LIO**:
```bash
cd ~/Tank_projects/tank_ws
source install/setup.bash
ros2 launch point_lio mapping_unilidar_l2.launch.py
```

### 3. Monitor Performance

**Check if LiDAR data is flowing**:
```bash
ros2 topic hz /unilidar/cloud
# Should show ~10 Hz for L2
```

**Check IMU data**:
```bash
ros2 topic hz /unilidar/imu
# Should show ~200 Hz
```

**Check Point-LIO odometry output**:
```bash
ros2 topic echo /Odometry --no-arr
```

**List all topics**:
```bash
ros2 topic list
```

### 4. Visualize TF Tree

```bash
ros2 run tf2_tools view_frames
# Generates frames.pdf showing the transform tree
```

## Configuration Files

All config files are located at:
```
~/Tank_projects/tank_ws/src/point_lio/config/
```

**For L2 LiDAR**: `unilidar_l2.yaml`

Key parameters you might want to adjust:
```yaml
# Topic names
lid_topic: "/unilidar/cloud"
imu_topic: "/unilidar/imu"

# Frame names (must match your TF tree)
map_frame: "map"
baselink_frame: "base_link"  
laser_frame: "lidar_link"

# Point cloud filtering (adjust for performance vs quality)
filter_size_surf: 0.5  # Smaller = denser map, more CPU
filter_size_map: 0.5

# IMU settings
acc_norm: 9.81  # Gravity in m/s²
```

## Output Topics

Point-LIO publishes:
- `/Odometry` - Robot pose (nav_msgs/Odometry)
- `/cloud_registered` - Registered point cloud (sensor_msgs/PointCloud2)
- `/cloud_registered_body` - Point cloud in body frame
- `/path` - Trajectory path (nav_msgs/Path)
- TF: `map → base_link`

## Saving Maps

To save point cloud maps, edit the config file:
```yaml
pcd_save:
  pcd_save_en: true
  interval: -1  # -1 = save all at end, >0 = save every N scans
```

Maps are saved to: `~/Tank_projects/tank_ws/src/point_lio/PCD/scans.pcd`

View saved map:
```bash
pcl_viewer ~/Tank_projects/tank_ws/src/point_lio/PCD/scans.pcd
```

## Troubleshooting

### No LiDAR data
```bash
# Check if driver is running
ros2 topic list | grep unilidar

# Check topic frequency
ros2 topic hz /unilidar/cloud
```

### IMU initialization stuck at low percentage
- Keep the robot **completely stationary** during initialization
- Wait 5-10 seconds for initialization to complete
- Check IMU topic: `ros2 topic echo /unilidar/imu`

### Build errors
```bash
# Clean rebuild
cd ~/Tank_projects/tank_ws
rm -rf build/point_lio install/point_lio
source /opt/ros/humble/setup.bash
colcon build --packages-select point_lio --symlink-install
```

### TF errors
Check that frame names in config match your robot's TF tree:
```bash
ros2 run tf2_ros tf2_echo map base_link
```

## Useful Scripts

### Reinstall Point-LIO
```bash
cd ~/Tank_projects/tank_ws
./reinstall_point_lio.sh
```

### Verify Installation
```bash
cd ~/Tank_projects/tank_ws
./test_point_lio.sh
```

## Performance Tips

### For Better Performance (less dense map)
```yaml
filter_size_surf: 0.5
filter_size_map: 0.5
```

### For Better Quality (denser map, more CPU)
```yaml
filter_size_surf: 0.1
filter_size_map: 0.1
```

### Disable RViz for Speed
```bash
ros2 launch point_lio mapping_unilidar_l2.launch.py rviz:=false
```

## Integration with Your Tank

Point-LIO is already integrated into your tank system via:
- `tank_localization` package
- `tank_bringup/launch/sensors_localization.launch.py`

To use in your full system:
```bash
cd ~/Tank_projects/tank_ws
source install/setup.bash
ros2 launch tank_bringup sensors_localization.launch.py
```

## Documentation

- **Full Setup Guide**: `POINT_LIO_SETUP_GUIDE.md`
- **GitHub ROS2 Port**: https://github.com/dfloreaa/point_lio_ros2
- **Original Algorithm**: https://github.com/hku-mars/Point-LIO

## Support

For issues:
1. Check the troubleshooting section above
2. Review logs: `~/.ros/log/latest/`
3. Check GitHub issues: https://github.com/dfloreaa/point_lio_ros2/issues

---

**Status**: ✅ Installed and Verified
**Date**: December 2025
**LiDAR**: Unitree L2
**ROS2**: Humble (Ubuntu 22.04)

