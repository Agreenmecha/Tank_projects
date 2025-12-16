# URDF and LiDAR Frame Setup Guide

## Overview
This document covers the complete setup of the `master_bot` URDF for ROS 2 and the integration of dual Unitree L2 LiDARs with proper frame alignment for visualization and mapping.

---

## URDF Modifications

### 1. Base Footprint Link
**Problem**: The original URDF had `base_link` as the root, positioned at the top of the robot, causing the robot to appear below the ground plane in RViz.

**Solution**: Added a `base_footprint` link at ground level (Z=0) with `base_link` offset 0.5m above it.

```xml
<!-- Base footprint link at ground level -->
<link name="base_footprint">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <box size="0.001 0.001 0.001" />
    </geometry>
  </visual>
</link>

<!-- Joint connecting base_footprint to base_link -->
<joint name="base_footprint_joint" type="fixed">
  <origin xyz="0 0 0.5" rpy="0 0 0" /> <!-- Offset base_link 0.5m up from ground -->
  <parent link="base_footprint" />
  <child link="base_link" />
</joint>
```

### 2. LiDAR Links (l_FL2 and l_BL2)
**Original URDF Links**:
- `l_FL2`: Front LiDAR mounting position
- `l_BL2`: Rear LiDAR mounting position

**Important Note**: During setup, we discovered the physical LiDAR data was swapped (front sensor data appearing at rear position). The URDF link names were swapped to match the physical sensor outputs:
- Physical **front** sensor (192.168.123.124) → publishes to `l_FL2` frame (originally rear position in URDF)
- Physical **rear** sensor (192.168.123.123) → publishes to `l_BL2` frame (originally front position in URDF)

The original geometric positions were kept; only the names were swapped.

---

## LiDAR Frame Fixing Problem

### The Challenge
The Unitree L2 LiDAR driver (`unitree_lidar_ros2`) has built-in IMU-based orientation correction. It publishes:
1. **Point clouds** with IMU-fused positioning (dynamic TF chain)
2. **IMU data** separately for localization/filtering

**The Problem**:
- Driver creates its own TF chain: `{lidar}_imu_initial` → `{lidar}_imu` → `{lidar}`
- This dynamic TF chain conflicts with the URDF's static transforms
- Point clouds would "orbit" or move independently from the robot model
- Couldn't directly connect URDF links to driver's IMU frames without conflicts

**Our Goal**:
- Keep IMU data available for sensor fusion/localization
- Have point clouds appear fixed to the robot model for visualization and mapping
- Maintain separate, independently adjustable rotations for front and rear lidars

---

## Solution: Point Cloud Frame Fixer Node

### Architecture
Created a ROS 2 Python node (`pointcloud_frame_fixer.py`) that:
1. **Subscribes** to raw driver point clouds (with driver's default frames)
2. **Applies** geometric rotation transformation to the point cloud data
3. **Republishes** with URDF frame IDs for static visualization

### Data Flow
```
Unitree L2 Driver                 Frame Fixer Node              RViz/Mapping
────────────────                 ─────────────────             ─────────────
/lidar_front/cloud    ──────►    Transform +      ──────►    /lidar_front/cloud_fixed
(frame: unilidar_lidar)           Rotate -115°                (frame: l_FL2)
                                  Set frame: l_FL2

/lidar_rear/cloud     ──────►    Transform +      ──────►    /lidar_rear/cloud_fixed
(frame: unilidar_lidar)           Rotate -25°                 (frame: l_BL2)
                                  Set frame: l_BL2

/lidar_front/imu      ──────────────────────────────────►    (unchanged, for localization)
/lidar_rear/imu       ──────────────────────────────────►    (unchanged, for localization)
```

### Key Features
- **Independent rotations**: Each lidar has its own rotation parameter
- **Dynamic adjustment**: Parameters can be changed at runtime without restart
- **Z-axis rotation**: Rotates X-Y plane around Z-axis (CCW positive)
- **Preserves IMU data**: IMU topics remain untouched for sensor fusion

---

## Configuration Details

### LiDAR Network Settings
**In `lidar_dual.launch.py`**:

| Parameter | Front LiDAR | Rear LiDAR |
|-----------|-------------|------------|
| LiDAR IP | 192.168.123.124 | 192.168.123.123 |
| LiDAR Port | 6101 | 6101 |
| Local Port | 6201 | 6202 |
| Range Min | 0.05m | 0.3m (filters robot body) |
| Range Max | 30.0m | 30.0m |

### Frame Configuration
| LiDAR | Driver Frame | URDF Frame | Published Topic | Fixed Topic |
|-------|--------------|------------|-----------------|-------------|
| Front | `unilidar_lidar` | `l_FL2` | `/lidar_front/cloud` | `/lidar_front/cloud_fixed` |
| Rear | `unilidar_lidar` | `l_BL2` | `/lidar_rear/cloud` | `/lidar_rear/cloud_fixed` |

### Rotation Settings
**Optimal values determined through testing**:
- **Front LiDAR**: `-115.0°` (115° clockwise)
- **Rear LiDAR**: `-25.0°` (25° clockwise)

These rotations correct for the physical mounting orientation of the sensors.

---

## Usage

### 1. Launch Hardware
```bash
cd ~/Tank_projects/tank_ws
source install/setup.bash
ros2 launch tank_sensors hardware.launch.py
```

This launches:
- GNSS
- Dual LiDARs (front + rear)
- Point cloud frame fixer
- Robot state publisher
- Camera (if enabled)

### 2. Visualize in RViz (Remote Desktop)
```bash
# Set ROS 2 domain and DDS implementation
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

cd ~/Tank_projects/tank_ws
source install/setup.bash
rviz2
```

**Add to RViz**:
- Fixed Frame: `base_footprint`
- Add: `RobotModel`
- Add: `PointCloud2` → Topic: `/lidar_front/cloud_fixed`
- Add: `PointCloud2` → Topic: `/lidar_rear/cloud_fixed`

### 3. Adjust Rotation (if needed)
```bash
# Front lidar
ros2 param set /pointcloud_frame_fixer front_rotation_degrees -115.0

# Rear lidar
ros2 param set /pointcloud_frame_fixer rear_rotation_degrees -25.0
```

Changes apply immediately without restart.

### 4. Check Topics
```bash
# Available topics
ros2 topic list | grep lidar

# Expected output:
# /lidar_front/cloud          (driver raw output)
# /lidar_front/cloud_fixed    (fixed frame, for RViz)
# /lidar_front/imu            (IMU data)
# /lidar_rear/cloud           (driver raw output)
# /lidar_rear/cloud_fixed     (fixed frame, for RViz)
# /lidar_rear/imu             (IMU data)
```

---

## Technical Implementation

### Point Cloud Transformation
The `pointcloud_frame_fixer.py` node applies a 2D rotation around the Z-axis:

```python
# Rotation matrix (CCW around Z)
cos_θ = cos(rotation_rad)
sin_θ = sin(rotation_rad)

x_rotated = x * cos_θ - y * sin_θ
y_rotated = x * sin_θ + y * cos_θ
z_unchanged = z
```

### Dynamic Parameters
Parameters are declared with `ParameterDescriptor` to enable runtime modification:

```python
self.declare_parameter('front_rotation_degrees', -115.0, 
    ParameterDescriptor(
        type=ParameterType.PARAMETER_DOUBLE,
        description='Rotation for front lidar (CCW around Z)',
        read_only=False
    ))
```

---

## Troubleshooting

### LiDAR Point Clouds Not Visible
1. **Check TF tree**: `ros2 run tf2_tools view_frames`
   - Should show: `world` → `base_footprint` → `base_link` → `l_FL2` / `l_BL2`
2. **Check topics**: `ros2 topic hz /lidar_front/cloud_fixed`
3. **Check frame IDs**: `ros2 topic echo /lidar_front/cloud_fixed --once | grep frame_id`

### Point Clouds Orbiting/Moving
- **Cause**: Using raw driver topics instead of `cloud_fixed` topics
- **Fix**: In RViz, use `/lidar_front/cloud_fixed` and `/lidar_rear/cloud_fixed`

### LiDAR Seeing Robot Body
- **Cause**: `range_min` too small
- **Fix**: Increase `range_min` in `lidar_dual.launch.py` (currently 0.3m for rear)
- **Dynamic test**: `ros2 param set /lidar_rear/lidar_rear range_min 0.5`

### UDP Port Bind Errors
- **Symptom**: `[UDPHandler] bind udp port failed`
- **Cause**: Old lidar driver processes still running
- **Fix**:
  ```bash
  pkill -9 -f unitree_lidar_ros2
  pkill -9 -f hardware
  sudo lsof -i :6201
  sudo lsof -i :6202
  # Kill any remaining processes, then relaunch
  ```

### Incorrect Point Cloud Orientation
- **Symptom**: Point clouds upside down or rotated wrong
- **Fix**: Adjust rotation parameters:
  ```bash
  ros2 param set /pointcloud_frame_fixer front_rotation_degrees <new_value>
  ros2 param set /pointcloud_frame_fixer rear_rotation_degrees <new_value>
  ```

---

## File Locations

### URDF Files
- Main URDF: `tank_ws/src/master_bot/urdf/master_bot.urdf`
- Launch files: `tank_ws/src/master_bot/launch/`
  - `display.launch.py` - RViz visualization
  - `gazebo.launch.py` - Gazebo Fortress simulation

### LiDAR Driver Files
- Launch file: `tank_ws/src/tank_sensors/launch/lidar_dual.launch.py`
- Frame fixer: `tank_ws/src/tank_sensors/src/pointcloud_frame_fixer.py`
- Hardware launch: `tank_ws/src/tank_sensors/launch/hardware.launch.py`

### Package Files
- `tank_ws/src/tank_sensors/package.xml` - Dependencies
- `tank_ws/src/tank_sensors/CMakeLists.txt` - Build configuration

---

## Testing Procedure

### 1. Initial Setup Test
```bash
# On rover
ros2 launch tank_sensors hardware.launch.py

# On desktop
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 node list  # Should see lidar nodes
ros2 topic list | grep cloud  # Should see cloud_fixed topics
```

### 2. Frame Verification
```bash
# Check published frames
ros2 topic echo /lidar_front/cloud_fixed --once | grep frame_id
# Expected: frame_id: l_FL2

ros2 topic echo /lidar_rear/cloud_fixed --once | grep frame_id
# Expected: frame_id: l_BL2
```

### 3. RViz Visualization
1. Launch RViz with ROS_DOMAIN_ID=42
2. Set Fixed Frame: `base_footprint`
3. Add RobotModel
4. Add both `/lidar_front/cloud_fixed` and `/lidar_rear/cloud_fixed`
5. Verify point clouds align with robot model and don't orbit

### 4. Rotation Calibration
If point clouds appear misaligned:
1. Identify rotation error (how many degrees off?)
2. Adjust dynamically: `ros2 param set /pointcloud_frame_fixer front_rotation_degrees <value>`
3. When optimal, update `lidar_dual.launch.py` defaults
4. Commit and push changes

---

## Summary of Changes from Original

1. **URDF**: Added `base_footprint` link and adjusted `base_link` offset
2. **LiDAR Integration**: Created frame fixer to decouple point clouds from IMU-fused frames
3. **Network Configuration**: Set up dual LiDAR with separate UDP ports
4. **Rotation Calibration**: Applied -115° and -25° rotations for front/rear alignment
5. **Dynamic Parameters**: Enabled runtime adjustment of rotations
6. **Range Filtering**: Increased rear lidar `range_min` to 0.3m to filter robot body

---

## Future Improvements

1. **Robot Self Filter**: Implement proper URDF-based point cloud filtering
2. **Sensor Fusion**: Integrate LiDAR IMU data with other sensors for localization
3. **Dynamic Reconfigure**: Add GUI for easier parameter adjustment
4. **Auto-calibration**: Automatic rotation calibration using known features
5. **Point Cloud Filtering**: Add voxel filtering, outlier removal for cleaner clouds

---

## Contact & Support

- GitHub: `Agreenmecha/Tank_projects`
- Branch: `phase1-complete`
- ROS 2 Version: Humble
- Tested on: NVIDIA Jetson Orin Nano (rover) + Ubuntu Desktop (visualization)

Last Updated: December 2025

