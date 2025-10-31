# CAD Model to URDF Conversion Guide

This guide helps you convert your tank CAD model to a ROS2 URDF for navigation and sensor integration.

---

## 📐 What You Need to Export from CAD

### 1. **Coordinate System Setup**

Before exporting, establish your coordinate frame:

**ROS Convention (Right-Hand Rule):**
- **X-axis:** Forward (front of tank)
- **Y-axis:** Left (driver's left side)
- **Z-axis:** Up (vertical)

**Origin (base_link):**
- Place at **center of tank** at ground level (or CG height if preferred)
- Typically: midpoint between tracks, at center of wheelbase

---

## 🎨 Mesh Export (For Visualization in RViz)

### Visual Meshes (High Detail)

Export as **STL** or **DAE (COLLADA)** format:

```
meshes/visual/
├── tank_body.stl          # Main chassis
├── left_track.stl         # Left track (for animation)
├── right_track.stl        # Right track (for animation)
├── front_lidar.stl        # (optional) L2 visual
├── rear_lidar.stl         # (optional)
└── camera.stl             # (optional) e-CAM25 visual
```

**Export Settings:**
- **Format:** STL (binary) or COLLADA (.dae with textures)
- **Units:** Meters
- **Origin:** Match your base_link origin
- **Resolution:** Medium (don't need CAD-level detail)
- **File size:** Keep under 5 MB per mesh for RViz performance

### Collision Meshes (Low Poly)

Simplified geometry for collision checking:

```
meshes/collision/
├── tank_body_collision.stl    # Simplified hull
├── left_track_collision.stl   # Track footprint
└── right_track_collision.stl  # Track footprint
```

**Export Settings:**
- **Format:** STL
- **Simplification:** High (use bounding boxes or simplified hulls)
- **Goal:** < 1000 triangles total for performance

**Alternative:** Use primitive shapes in URDF (box, cylinder) instead of mesh

---

## 📏 Critical Measurements Needed

Measure these from your CAD model (in **meters**):

### Tank Dimensions

| Measurement | Value | Notes |
|-------------|-------|-------|
| **Track width (center-to-center)** | 0.600m | Distance between track centers ✅ |
| **Track contact length** | 0.410m | Ground contact patch ✅ |
| **Overall length** | ? m | Including front/rear overhang |
| **Overall width** | ? m | Outer edge to outer edge |
| **Overall height** | ? m | Top of highest component |
| **CG height** | 0.229m | Center of gravity above ground ✅ |

### Sensor Mounting Positions (from base_link origin)

**Front L2 LiDAR:**
```yaml
position:
  x: ? m      # Forward from base_link
  y: ? m      # Left/right offset (0 if centered)
  z: ? m      # Height above base_link
orientation:
  roll: 0°
  pitch: ? °  # Tilt (0° = level, negative = down)
  yaw: 0°
```

**Rear L2 LiDAR:**
```yaml
position:
  x: ? m      # Negative if behind base_link
  y: ? m
  z: ? m
orientation:
  roll: 0°
  pitch: ? °
  yaw: 180°   # Facing backward
```

**e-CAM25 Camera:**
```yaml
position:
  x: ? m      # Typically front of tank
  y: ? m
  z: ? m
orientation:
  roll: 0°
  pitch: ? °  # Typically tilted down 10-30° for terrain viewing
  yaw: 0°
```

**ZED-F9P GNSS Antenna:**
```yaml
position:
  x: ? m
  y: ? m
  z: ? m      # Typically highest point for clear sky view
orientation:
  roll: 0°
  pitch: 0°
  yaw: 0°
```

---

## 🤖 URDF Structure Template

### Your URDF will have this structure:

```xml
tank.urdf.xacro
├── Base Link (tank center)
├── Visual Geometry (meshes)
├── Collision Geometry (simplified)
├── Inertial Properties (mass, inertia)
│
├── Sensor Frames:
│   ├── front_lidar_link (L2 position)
│   │   └── front_lidar_imu_link (IMU offset)
│   ├── rear_lidar_link
│   │   └── rear_lidar_imu_link
│   ├── camera_link (e-CAM25)
│   │   └── camera_optical_frame (ROS camera convention)
│   ├── gnss_link (antenna position)
│   │
│   └── Track Links:
│       ├── left_track_link
│       └── right_track_link
```

---

## 🛠️ CAD → URDF Workflow

### Step 1: Export Meshes

1. **Set origin** in CAD to tank center (base_link)
2. **Export body** as STL (visual + collision)
3. **Export tracks** separately (for potential animation)
4. **Simplify collision meshes** (or just use boxes)
5. Save to `tank_description/meshes/`

### Step 2: Measure Sensor Positions

1. **Create coordinate system** at base_link origin
2. **Measure each sensor** position (X, Y, Z)
3. **Measure orientations** (especially camera tilt, rear LiDAR 180° yaw)
4. **Document** in spreadsheet or notes

### Step 3: Create URDF

Create `tank_description/urdf/tank.urdf.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="tank">
  
  <!-- Parameters from CAD measurements -->
  <xacro:property name="track_width" value="0.600"/>
  <xacro:property name="track_length" value="0.410"/>
  <xacro:property name="body_mass" value="30"/> <!-- kg, adjust based on actual -->
  
  <!-- Base link (tank center) -->
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="package://tank_description/meshes/visual/tank_body.stl"/>
      </geometry>
      <material name="tank_gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    
    <collision>
      <geometry>
        <box size="0.600 0.500 0.300"/> <!-- Simplified bounding box -->
      </geometry>
    </collision>
    
    <inertial>
      <mass value="${body_mass}"/>
      <origin xyz="0 0 0.229"/> <!-- CG at 229mm height -->
      <inertia ixx="1.0" ixy="0.0" ixz="0.0"
               iyy="1.0" iyz="0.0"
               izz="1.0"/> <!-- Calculate from CAD or use rough estimates -->
    </inertial>
  </link>
  
  <!-- Front L2 LiDAR -->
  <joint name="base_to_front_lidar" type="fixed">
    <parent link="base_link"/>
    <child link="front_lidar_link"/>
    <origin xyz="0.250 0.000 0.300" rpy="0 0 0"/> <!-- REPLACE WITH YOUR MEASUREMENTS -->
  </joint>
  
  <link name="front_lidar_link"/>
  
  <!-- Add more sensors... -->
  
</robot>
```

### Step 4: Test URDF

```bash
# Check URDF syntax
check_urdf tank.urdf.xacro

# Visualize in RViz
ros2 launch tank_description view_robot.launch.py
```

---

## 📦 What to Place in tank_description/

```
tank_description/
├── urdf/
│   └── tank.urdf.xacro           # Main robot description (YOU CREATE)
├── meshes/
│   ├── visual/
│   │   ├── tank_body.stl         # Export from CAD
│   │   ├── left_track.stl
│   │   └── right_track.stl
│   └── collision/
│       └── tank_body_collision.stl
├── config/
│   └── joint_state_publisher.yaml (if needed)
├── launch/
│   ├── robot_state_publisher.launch.py  # Auto-created
│   └── view_robot.launch.py             # For testing in RViz
└── CAD_TO_URDF_GUIDE.md (this file)
```

---

## 🎯 Key Points for Your Tank

### Tracked Vehicle Specifics

1. **No wheel joints** - tracks are typically modeled as fixed links
2. **Footprint** is important for DWA planner:
   ```python
   footprint: [
     [0.300, 0.300],   # Front-right
     [0.300, -0.300],  # Front-left
     [-0.250, -0.300], # Rear-left
     [-0.250, 0.300]   # Rear-right
   ]
   ```

3. **Low CG (229mm)** - excellent for stability, note in URDF inertial

4. **Dual LiDAR setup** - ensure rear LiDAR has `yaw="3.14159"` (180°)

5. **Camera tilt** - likely tilted down 10-30° for terrain viewing

---

## 🔧 CAD Software Specific Tips

### SolidWorks
- Use "Export → STL" or "Save As → STL"
- Set units to meters
- Export with coordinate system aligned to base_link

### Fusion 360
- Right-click body → "Save as STL"
- Set refinement to "Medium"
- Use "Export" for STEP/IGES if needed

### Onshape
- Right-click part → "Export"
- Choose STL format
- Set units to meters

### FreeCAD
- Select body → "File → Export → Mesh formats"
- Choose STL binary

---

## ✅ URDF Creation Checklist

- [ ] Define base_link origin in CAD (tank center)
- [ ] Export visual meshes (STL/DAE) in meters
- [ ] Export simplified collision meshes (or use primitives)
- [ ] Measure all sensor positions (X, Y, Z)
- [ ] Measure sensor orientations (roll, pitch, yaw)
- [ ] Create tank.urdf.xacro with all sensors
- [ ] Add inertial properties (mass, inertia, CG)
- [ ] Test with `check_urdf`
- [ ] Visualize in RViz
- [ ] Verify TF tree is correct (`ros2 run tf2_tools view_frames`)

---

## 📚 Resources

- **URDF Tutorial:** http://wiki.ros.org/urdf/Tutorials
- **Xacro Guide:** http://wiki.ros.org/xacro
- **TF2 Tutorial:** https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html
- **SolidWorks to URDF:** http://wiki.ros.org/sw_urdf_exporter (ROS1, but concepts apply)

---

## 🚀 After URDF is Ready

Once you create the URDF, we'll:
1. Create `robot_state_publisher.launch.py`
2. Integrate into `tank_bringup`
3. Test TF tree
4. Configure costmaps with footprint
5. Set up RViz config

**Let me know when your URDF is ready, and I'll help integrate it!** 🤖

