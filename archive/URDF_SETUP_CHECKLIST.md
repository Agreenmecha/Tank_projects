# URDF Setup Checklist

**Status:** Ready for URDF configuration  
**Target:** Tank rover with ODrive motors, sensors, and differential drive

---

## 📋 What You Need

### 1. Tank Geometry & Dimensions
- [ ] **Track width** (distance between wheels) - You have: 0.60m (600mm)
- [ ] **Wheel radius** - You have: 0.10m (adjust to actual)
- [ ] **Tank length** (front to back)
- [ ] **Tank height** (ground to top)
- [ ] **Wheelbase** (distance from front to rear wheel contact)
- [ ] **Ground clearance**
- [ ] **Center of mass location** (optional but helpful)

### 2. Sensor Positions (from CAD)
- [ ] **Front LiDAR** (L2) - X, Y, Z position relative to center
- [ ] **Rear LiDAR** - X, Y, Z position
- [ ] **e-CAM25 camera** - X, Y, Z position
- [ ] **IMU** (if any) - X, Y, Z position
- [ ] **u-blox GNSS** - X, Y, Z position (antenna height)

### 3. Motor Configuration
- [ ] **Motor/wheel attachment points** - X, Y, Z for each wheel
- [ ] **Motor frames** - Usually: left_wheel_link, right_wheel_link
- [ ] **Track/drive type** - Differential drive (two motors)

### 4. Mesh Files (Optional but Recommended)
- [ ] **Tank body mesh** (STL or DAE format)
- [ ] **Wheel meshes** (optional)
- [ ] **Sensor mounts** (optional)
- [ ] **Track/tread visualization** (optional)

---

## 🛠️ Three Approaches

### Approach 1: Quick URDF (Fastest - Recommended for Testing)
**Time:** 30 minutes

Create a basic XACRO file with:
- Base link (tank body as box)
- Left/right wheel links
- Sensor frames
- NO mesh files (use geometric shapes)

**Pros:** Quick, works immediately  
**Cons:** Not as detailed visually

**File:** `urdf/tank.urdf.xacro`

---

### Approach 2: CAD Export URDF (Professional)
**Time:** 1-2 hours

Export from CAD software (SolidWorks, FreeCAD, etc.):
- CAD → URDF converter
- Generate meshes automatically
- Import collision geometry

**Tools:**
- SolidWorks: SW2URDF plugin
- FreeCAD: Built-in URDF export
- Fusion 360: Community scripts available

**Pros:** Accurate, detailed  
**Cons:** Requires CAD knowledge, larger files

---

### Approach 3: Hybrid (Best Quality)
**Time:** 2-3 hours

- Export meshes from CAD (visual only)
- Hand-write XACRO for structure
- Link meshes to URDF

**Pros:** Best of both worlds  
**Cons:** More setup work

---

## 📁 Current ROS2 Structure

```
tank_ws/src/tank_description/
├── CMakeLists.txt
├── package.xml
├── CAD_TO_URDF_GUIDE.md          # Detailed guide
├── launch/
│   ├── robot_state_publisher.launch.py
│   └── view_robot.launch.py       # View in RViz
├── urdf/                           # WHERE YOUR URDF GOES
│   └── tank.urdf.xacro           # Main URDF file
└── meshes/                         # WHERE MESHES GO
    ├── visual/
    │   ├── tank_body.stl
    │   └── wheels.stl
    └── collision/
        └── tank_simple.stl
```

---

## 🚀 Next Steps (Pick One)

### Quick Start (Approach 1 - Recommended)
```bash
# Create quick URDF without meshes
# We'll create tank.urdf.xacro with your dimensions
```

### From CAD Model (Approach 2)
```bash
# Export from your CAD software
# Need: Mesh files (STL/DAE) + dimensions
```

### Do You Have?
- [ ] CAD file (SolidWorks, STEP, FreeCAD, etc.)?
- [ ] Mesh files already exported?
- [ ] Just dimensions/drawings?

---

## 📐 URDF Basics

### Minimal URDF Template

```xml
<?xml version="1.0"?>
<robot name="tank" xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <!-- Base Link (main chassis) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.6 0.4 0.2"/>  <!-- length width height -->
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
    </collision>
  </link>

  <!-- Left Wheel -->
  <link name="left_wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
  </link>

  <!-- Left Wheel Joint -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel_link"/>
    <origin xyz="0 0.3 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Right Wheel (mirror) -->
  <link name="right_wheel_link">...</link>
  <joint name="right_wheel_joint" type="continuous">...</joint>

  <!-- Sensors -->
  <link name="front_lidar_frame"/>
  <joint name="front_lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="front_lidar_frame"/>
    <origin xyz="0.25 0 0.1" rpy="0 0 0"/>
  </joint>

</robot>
```

---

## 💡 Questions to Answer

Before we proceed, please provide:

1. **Do you have CAD files?**
   - [ ] Yes → Format? (STEP, SolidWorks, FreeCAD, etc.)
   - [ ] Mesh files only (STL/DAE)
   - [ ] Just dimensions/sketches

2. **Tank Dimensions:**
   - Length: _____ meters
   - Width: _____ meters
   - Height: _____ meters
   - Wheelbase: _____ meters
   - Wheel radius: 0.10m ✓

3. **Sensor Mounting:**
   - Front LiDAR position: X=___ Y=___ Z=___
   - Rear LiDAR position: X=___ Y=___ Z=___
   - Camera position: X=___ Y=___ Z=___
   - GNSS antenna height: ___ meters

---

## ✅ Quick URDF Creation (If No CAD Available)

We can create a URDF in 10 minutes using:
- Your known dimensions (600mm track width, 100mm wheels)
- Standard tank proportions
- Simple geometric shapes

**Just tell me:** What's your tank's length and height?

---

## 🎯 What Happens After URDF

Once URDF is created, you can:

1. **View in RViz**
   ```bash
   ros2 launch tank_description view_robot.launch.py
   ```

2. **Simulate movements**
   - Test sensor frame transformations
   - Verify coordinate frames

3. **Integrate with full stack**
   - tf2 transforms
   - Sensor fusion
   - Navigation

---

## 📚 Reference

- Guide: `CAD_TO_URDF_GUIDE.md` (in tank_description folder)
- ROS2 URDF Docs: https://docs.ros.org/en/humble/Tutorials/URDF/URDF-Main.html
- XACRO Guide: https://wiki.ros.org/xacro

---

**What would you like to do?**

- [ ] **Quick URDF** - Create basic URDF with standard dimensions
- [ ] **Import CAD** - You have mesh files ready
- [ ] **Get dimensions** - I'll create URDF once you provide them

