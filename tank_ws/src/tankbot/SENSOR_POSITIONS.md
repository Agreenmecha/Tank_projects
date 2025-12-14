# Sensor Position Adjustment Guide

All sensor positions in the URDF are relative to `base_link` origin. Adjust the `xyz` and `rpy` values in the URDF based on your CAD measurements.

## Current Placeholder Positions

### Front LiDAR (lidar_front)
```xml
<origin xyz="0.25 0 0.15" rpy="0 0 0" />
```
- **x=0.25m**: 25cm forward from base_link origin
- **y=0**: Centered (left/right)
- **z=0.15m**: 15cm above base_link origin
- **rpy**: Level, facing forward

**To adjust:** Measure from base_link origin to center of front LiDAR in your CAD model.

---

### Rear LiDAR (lidar_rear)
```xml
<origin xyz="-0.25 0 0.15" rpy="0 0 3.14159" />
```
- **x=-0.25m**: 25cm rearward (negative x) from base_link origin
- **y=0**: Centered
- **z=0.15m**: 15cm above base_link origin
- **rpy**: Yaw=180° (3.14159 rad) to face rearward

**To adjust:** Measure rear LiDAR position, set yaw to 180° (π radians).

---

### Camera (camera_link)
```xml
<origin xyz="0.20 0 0.10" rpy="0 -0.35 0" />
```
- **x=0.20m**: 20cm forward from base_link
- **y=0**: Centered
- **z=0.10m**: 10cm above base_link origin
- **rpy**: Pitch=-0.35 rad (~-20°) tilted down for terrain viewing

**To adjust:** 
- Measure camera position
- Adjust pitch (second value in rpy) for tilt angle
- Negative pitch = tilted down (typical for ground vehicles)

---

### Camera Optical Frame (camera_optical_frame)
```xml
<origin xyz="0 0 0" rpy="-1.5708 0 -1.5708" />
```
**Do not change this!** This is the standard ROS camera optical frame transformation:
- Z-axis points forward (optical axis)
- X-axis points right
- Y-axis points down

---

### GNSS Antenna (gnss)
```xml
<origin xyz="0.0 0 0.25" rpy="0 0 0" />
```
- **x=0**: Centered front/back
- **y=0**: Centered left/right
- **z=0.25m**: 25cm above base_link origin (top of robot)
- **rpy**: Level, facing sky

**To adjust:** Position at highest point on robot for clear sky view. Usually centered or slightly forward.

---

## Coordinate System

**base_link convention:**
- **X**: Forward (+x) / Backward (-x)
- **Y**: Left (+y) / Right (-y) 
- **Z**: Up (+z) / Down (-z)

**Rotations (rpy):**
- **roll**: Rotation around X-axis
- **pitch**: Rotation around Y-axis (positive = nose up)
- **yaw**: Rotation around Z-axis (positive = counterclockwise when viewed from above)

## How to Measure in CAD

1. **Set coordinate system** at base_link origin (center of chassis)
2. **Measure each sensor** position relative to base_link:
   - X = distance forward/backward
   - Y = distance left/right
   - Z = height above base_link
3. **Measure orientation**:
   - For LiDARs: Usually level (0,0,0) or rear-facing (0,0,π)
   - For camera: Usually tilted down (0, -10° to -30°, 0)
   - For GNSS: Always level (0,0,0)

## Editing the URDF

1. Open `tank_ws/src/tankbot/urdf/tankbot.urdf`
2. Find the sensor joint (search for "lidar_front", "lidar_rear", "camera_link", or "gnss")
3. Update the `<origin xyz="..." rpy="..." />` values
4. Rebuild: `cd ~/Tank_projects/tank_ws && colcon build --packages-select tankbot`
5. Relaunch and verify in RViz

## Verification in RViz

After adjusting positions:
1. Launch: `ros2 launch tankbot display.launch.py`
2. Add "TF" display to see all frames
3. Verify sensor frames appear at correct positions relative to base_link
4. Sensor pointclouds/camera images should align correctly in RViz

