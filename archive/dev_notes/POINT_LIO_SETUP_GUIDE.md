# Point-LIO ROS2 Setup Guide

## Overview
This guide covers the proper installation and configuration of Point-LIO for ROS2 Humble on Ubuntu 22.04, specifically configured for the Unitree L2 LiDAR.

## References
- **ROS2 Port**: https://github.com/dfloreaa/point_lio_ros2
- **Original ROS1**: https://github.com/unitreerobotics/point_lio_unilidar

## System Requirements

### Operating System
- Ubuntu 22.04 LTS
- ROS2 Humble (tested and working)

### Hardware
- Unitree L2 LiDAR (current setup)
- Also supports: L1, Livox Avia/Mid-360, Velodyne, Ouster

## Prerequisites

### 1. ROS2 Humble
Point-LIO requires ROS2 Humble. Verify installation:
```bash
ros2 --version
# Should show: ros2 doctor version
```

### 2. Required ROS2 Packages
```bash
sudo apt-get update
sudo apt-get install -y \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-visualization-msgs
```

### 3. Eigen3
```bash
sudo apt-get install -y libeigen3-dev
```

### 4. Unitree LiDAR SDK2 (for L2 LiDAR)
Located at: `/home/aaronjet/Tank_projects/tank_ws/src/external/unilidar_sdk2`

If not installed:
```bash
cd ~/Tank_projects/tank_ws/src/external
git clone https://github.com/unitreerobotics/unilidar_sdk2.git
cd ~/Tank_projects/tank_ws
colcon build --packages-select unitree_lidar_ros2 unitree_lidar_sdk --symlink-install
```

### 5. Livox ROS Driver 2 (Optional - only for Livox LiDARs)
Not required for Unitree LiDARs. Only install if using Livox Avia/Mid-360:
```bash
cd ~/Tank_projects/tank_ws/src/external
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd ~/Tank_projects/tank_ws
colcon build --packages-select livox_ros_driver2 --symlink-install
```

## Installation

### Quick Reinstallation (Automated)
```bash
cd ~/Tank_projects/tank_ws
./reinstall_point_lio.sh
```

### Manual Installation

1. **Clean previous build**:
```bash
cd ~/Tank_projects/tank_ws
rm -rf build/point_lio install/point_lio log/*/point_lio
```

2. **Source ROS2 and workspace**:
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash  # if already built other packages
```

3. **Build Point-LIO**:
```bash
colcon build --packages-select point_lio --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

4. **Source the workspace**:
```bash
source install/setup.bash
```

## Configuration

### For Unitree L2 LiDAR
Config file: `config/unilidar_l2.yaml`

Key parameters:
- `lid_topic`: `/unilidar/cloud` (default for L2)
- `imu_topic`: `/unilidar/imu` (built-in IMU)
- `lidar_type`: 5 (for Unitree L2)
- `scan_line`: 32
- `timestamp_unit`: 3 (seconds)

### Frame Configuration
Important for ROS2 TF tree integration:
- `map_frame`: "map"
- `baselink_frame`: "base_link"
- `laser_frame`: "lidar_link"

### IMU Settings
The L2 has a built-in IMU:
- `use_imu_as_input`: 1
- `imu_en`: true
- `extrinsic_est_en`: true (enables auto-calibration)

## Usage

### Launch Point-LIO

**For Unitree L2 LiDAR**:
```bash
# Terminal 1: Launch Point-LIO
cd ~/Tank_projects/tank_ws
source install/setup.bash
ros2 launch point_lio mapping_unilidar_l2.launch.py

# Optional: without RViz (faster)
ros2 launch point_lio mapping_unilidar_l2.launch.py rviz:=false
```

**For Unitree L1 LiDAR**:
```bash
ros2 launch point_lio mapping_unilidar_l1.launch.py
```

**For Livox Avia**:
```bash
# Terminal 1: Launch Livox driver
ros2 launch livox_ros_driver2 msg_HAP_launch.py

# Terminal 2: Launch Point-LIO
ros2 launch point_lio mapping_avia.launch.py
```

### Launch with LiDAR Driver

```bash
# Terminal 1: Launch L2 LiDAR driver
cd ~/Tank_projects/tank_ws
source install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py

# Terminal 2: Launch Point-LIO
cd ~/Tank_projects/tank_ws
source install/setup.bash
ros2 launch point_lio mapping_unilidar_l2.launch.py
```

### Check Topics

Verify LiDAR and IMU data:
```bash
# List all topics
ros2 topic list

# Check LiDAR point cloud
ros2 topic echo /unilidar/cloud --no-arr

# Check IMU data
ros2 topic echo /unilidar/imu

# Check Point-LIO odometry output
ros2 topic echo /Odometry

# Monitor TF tree
ros2 run tf2_tools view_frames
```

## Output Topics

Point-LIO publishes:
- `/Odometry` (nav_msgs/Odometry) - Robot pose estimation
- `/cloud_registered` (sensor_msgs/PointCloud2) - Registered point cloud
- `/cloud_registered_body` (sensor_msgs/PointCloud2) - Point cloud in body frame
- `/path` (nav_msgs/Path) - Trajectory path
- TF transforms: `map -> base_link`

## Saving Point Cloud Maps

To save PCD files, set in launch file or config:
```yaml
pcd_save:
  pcd_save_en: true
  interval: -1  # -1 saves all frames at end
```

PCD files saved to: `~/Tank_projects/tank_ws/src/point_lio/PCD/scans.pcd`

View saved map:
```bash
pcl_viewer ~/Tank_projects/tank_ws/src/point_lio/PCD/scans.pcd
```

## Troubleshooting

### Build Issues

**PCL not found**:
```bash
sudo apt-get install -y libpcl-dev
```

**Eigen not found**:
```bash
sudo apt-get install -y libeigen3-dev
```

**Custom PCL path** (if needed):
```bash
export PCL_ROOT=/path/to/custom/pcl
```

### Runtime Issues

**No LiDAR data**:
- Check if LiDAR driver is running: `ros2 topic list | grep unilidar`
- Verify topic names match in config file
- Check LiDAR connection: `ros2 topic hz /unilidar/cloud`

**IMU initialization stuck**:
- Keep robot stationary during initialization
- Check IMU topic: `ros2 topic echo /unilidar/imu`
- Verify `acc_norm` matches your IMU (default: 9.81 for m/s²)

**TF errors**:
- Verify frame names in config match your robot's TF tree
- Check with: `ros2 run tf2_ros tf2_echo map base_link`

**Poor mapping quality**:
- Adjust `filter_size_surf` and `filter_size_map` in config
- Check if moving too fast (aggressive motion)
- Verify extrinsic calibration between LiDAR and IMU

### Aggressive Motion Start
If starting from high-speed motion:
```yaml
start_in_aggressive_motion: true
gravity_init: [0.0, 9.810, 0.0]  # Adjust based on IMU mounting
```

## Performance Tuning

### For High-Speed Motion
```yaml
filter_size_surf: 0.5
filter_size_map: 0.5
cube_side_length: 1000.0
```

### For Dense Mapping
```yaml
filter_size_surf: 0.1
filter_size_map: 0.1
pcd_save_interval: 1
```

### CPU Performance
Point-LIO automatically uses multiprocessing on x86 with >3 cores. On ARM (Jetson), it uses single-threaded processing.

## Integration with Navigation Stack

Point-LIO provides odometry that can be used with:
1. **robot_localization** for sensor fusion
2. **Nav2** for autonomous navigation
3. Custom localization packages

Example integration in `tank_bringup/launch/sensors_localization.launch.py`

## File Structure

```
point_lio/
├── config/           # Configuration files for different LiDARs
│   ├── unilidar_l2.yaml
│   ├── unilidar_l1.yaml
│   ├── avia.yaml
│   └── ...
├── launch/          # Launch files
│   ├── mapping_unilidar_l2.launch.py
│   └── ...
├── src/             # Source code
│   ├── laserMapping.cpp
│   ├── Estimator.cpp
│   └── ...
├── include/         # Header files
│   ├── ikd-Tree/
│   ├── IKFoM/
│   └── ...
├── PCD/            # Saved point cloud maps
└── Log/            # Debug logs
```

## Performance Expectations

On Jetson Orin Nano:
- **CPU Usage**: ~150-200% (1.5-2 cores)
- **Memory**: ~500MB-1GB
- **Processing Rate**: Real-time at 10Hz for L2 LiDAR
- **Latency**: <100ms

## Additional Resources

- [Point-LIO Paper](https://github.com/hku-mars/Point-LIO)
- [Unitree L2 Manual](https://github.com/unitreerobotics/unilidar_sdk2)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)

## Support

For issues specific to:
- **ROS2 port**: https://github.com/dfloreaa/point_lio_ros2/issues
- **Original algorithm**: https://github.com/hku-mars/Point-LIO/issues
- **Unitree LiDAR**: https://github.com/unitreerobotics/unilidar_sdk2/issues

---

Last updated: December 2025

