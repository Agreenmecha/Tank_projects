# SolidWorks → OnShape → URDF Guide

**Goal:** Convert your SolidWorks tank assembly to URDF for ROS2  
**Tool:** OnShape's integrated URDF exporter  
**Estimated Time:** 30-45 minutes

---

## Step 1: Export from SolidWorks

### 1.1 Save Assembly as STEP
In SolidWorks:
```
File → Save As → Select format: STEP (*.step or *.stp)
```

**Important settings:**
- ✓ Include all parts
- ✓ Save as assembly (not individual parts)
- ✓ Use meter units (or remember to scale later)

**File:** `tank_assembly.step`

---

## Step 2: Import to OnShape

### 2.1 Create OnShape Account (if needed)
- Go to: https://www.onshape.com
- Sign up (free tier works for this)

### 2.2 Import Your Assembly
1. Click **"Create" → "Upload"**
2. Select your `tank_assembly.step` file
3. Wait for import to complete

### 2.3 Check the Model
- View in 3D to ensure all parts imported correctly
- Check that measurements look right
- Verify part locations relative to each other

---

## Step 3: Prepare for URDF Export

### 3.1 Fix Coordinate System (Important!)
Your base_link origin should be at the **tank center at ground level**.

**In OnShape:**
1. Right-click assembly → **"Edit assembly"**
2. Check part positions relative to origin
3. If needed, create a **coordinate system** at tank center:
   - Insert → Coordinate System
   - Place at tank center (ground level)
   - Name it: `base_link_origin`

### 3.2 Name Your Parts Clearly
Rename parts in OnShape to match ROS2 naming:
- `tank_body` or `chassis`
- `left_wheel`
- `right_wheel`
- `left_track` (optional)
- `right_track` (optional)
- `front_lidar_mount`
- `rear_lidar_mount`
- `camera_mount`
- `gnss_antenna`

**Why:** OnShape uses these names in the URDF - clear names = better URDF

---

## Step 4: Export URDF from OnShape

### 4.1 Access URDF Exporter
In OnShape assembly view:
1. Click the **three-dot menu** (top right)
2. Select **"Export URDF"** or **"Download as URDF"**
   - (Location may vary by OnShape version)

**Alternative method:**
- Right-click assembly → "Export" → Look for URDF option

### 4.2 URDF Export Settings

When the export dialog appears:

**Basic Settings:**
- [ ] **Base Link:** Select the main body (usually `tank_body`)
- [ ] **Units:** Should be **Meters** (verify!)
- [ ] **Include Meshes:** ✓ YES - This exports STL files
- [ ] **Mesh Format:** STL (recommended)
- [ ] **Collision Geometry:** Use simplified meshes

**Joint Configuration:**
- [ ] Set wheel joints as **continuous** (for rotation)
- [ ] Set sensor mounts as **fixed** joints
- [ ] Origin points should align with part connections

### 4.3 Download URDF Package
OnShape will generate a ZIP file containing:
```
tank_urdf.zip/
├── urdf/
│   └── tank_assembly.urdf
├── meshes/
│   ├── visual/
│   │   ├── tank_body.stl
│   │   ├── left_wheel.stl
│   │   ├── right_wheel.stl
│   │   └── ...
│   └── collision/
│       └── (simplified versions)
└── assembly.xml
```

**Download and extract** this to a temporary location.

---

## Step 5: Import into ROS2 Workspace

### 5.1 Directory Structure
Extract the ZIP and copy to your ROS2 workspace:

```bash
cd /home/aaronjet/Tank_projects/tank_ws/src/tank_description

# Create directories if they don't exist
mkdir -p urdf meshes/visual meshes/collision

# Copy URDF
cp ~/Downloads/tank_urdf/urdf/tank_assembly.urdf urdf/tank.urdf

# Copy meshes
cp -r ~/Downloads/tank_urdf/meshes/* meshes/
```

### 5.2 Convert to XACRO (Recommended)
XACRO files are more maintainable:

```bash
# Create XACRO version
cp urdf/tank.urdf urdf/tank.urdf.xacro
```

Then edit `urdf/tank.urdf.xacro` to add ROS parameters (optional but recommended).

---

## Step 6: Test in ROS2

### 6.1 Build Workspace
```bash
cd /home/aaronjet/Tank_projects/tank_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select tank_description
source install/setup.bash
```

### 6.2 View Robot in RViz
```bash
ros2 launch tank_description view_robot.launch.py
```

**You should see:**
- 3D tank model with meshes
- All parts in correct positions
- Coordinate frames visible

### 6.3 Check Coordinate Frames
```bash
# In another terminal
ros2 run tf2_tools view_frames.py
# Then open: /tmp/frames.pdf
```

---

## Step 7: Verify Joint Configuration

### 7.1 Check Wheel Joints
Your URDF should have wheel joints like:
```xml
<joint name="left_wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="left_wheel"/>
  <axis xyz="0 1 0"/>  <!-- Rotation around Y axis -->
</joint>
```

**If not continuous:** Edit URDF and change `type="revolute"` to `type="continuous"`

### 7.2 Check Sensor Frames
Sensor joints should be **fixed**:
```xml
<joint name="front_lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="front_lidar_frame"/>
  <origin xyz="0.3 0 0.15"/>  <!-- X Y Z position -->
</joint>
```

---

## Step 8: Fix Common Issues

### Issue 1: Wrong Units (Meshes too big/small)
**Symptom:** Robot appears huge or tiny in RViz

**Solution:**
1. Check SolidWorks units (should be meters)
2. Edit URDF and add scale factor if needed:
   ```xml
   <visual>
     <geometry>
       <mesh filename="package://tank_description/meshes/visual/tank_body.stl" scale="0.001 0.001 0.001"/>
     </geometry>
   </visual>
   ```
   (If meshes are in millimeters instead of meters)

### Issue 2: Incorrect Origin
**Symptom:** Robot floats or tilts in RViz

**Solution:**
1. Identify base_link origin point
2. Edit joints to correct positions:
   ```xml
   <origin xyz="X Y Z" rpy="0 0 0"/>
   ```

### Issue 3: Missing or Broken Meshes
**Symptom:** Parts show as wireframe or disappear

**Solution:**
```bash
# Check mesh files exist
ls -la urdf/../meshes/visual/

# Fix paths in URDF (should use package:// format)
# NOT: <mesh filename="/home/..."/>
# YES: <mesh filename="package://tank_description/meshes/visual/tank_body.stl"/>
```

---

## Step 9: Configure for Tank Control

### 9.1 Match ODrive Axes
Edit URDF to match your motor configuration:

```xml
<!-- Left motor (ODrive axis 0) -->
<joint name="left_wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="left_wheel"/>
  <axis xyz="0 1 0"/>  <!-- Y-axis rotation -->
  <limit effort="30" velocity="100"/>  <!-- Match ODrive limits -->
</joint>

<!-- Right motor (ODrive axis 1) -->
<joint name="right_wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="right_wheel"/>
  <axis xyz="0 1 0"/>
  <limit effort="30" velocity="100"/>
</joint>
```

### 9.2 Add Motor Transmission (Optional)
For simulation/control:
```xml
<transmission name="left_wheel_transmission">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_wheel_joint">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_motor">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

---

## Step 10: Final Verification

### Checklist:
- [ ] URDF imports without errors
- [ ] Robot displays correctly in RViz
- [ ] All parts visible with meshes
- [ ] Coordinate frames match expected positions
- [ ] Wheel joints are continuous
- [ ] Sensor frames properly positioned
- [ ] No floating/tilting in RViz
- [ ] Paths use `package://` format (not absolute paths)

---

## 📦 Final Structure

After completion, your workspace should look like:

```
tank_ws/src/tank_description/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── robot_state_publisher.launch.py
│   └── view_robot.launch.py
├── urdf/
│   ├── tank.urdf          ← URDF from OnShape
│   └── tank.urdf.xacro    ← (Optional) XACRO version
└── meshes/
    ├── visual/
    │   ├── tank_body.stl
    │   ├── left_wheel.stl
    │   ├── right_wheel.stl
    │   └── (other parts)
    └── collision/
        └── (simplified meshes)
```

---

## 🚀 Next Steps After URDF

Once URDF is working:

1. **Integrate ODrive control** with ROS2 node
2. **Mount sensor frames** (LiDAR, camera, GNSS)
3. **Test transformations** (tf2)
4. **Simulate movements** in RViz
5. **Test with real robot**

---

## 🔗 Useful Links

- **OnShape:** https://www.onshape.com
- **URDF Tutorial:** https://wiki.ros.org/urdf/Tutorials
- **ROS2 URDF Guide:** https://docs.ros.org/en/humble/Tutorials/URDF/URDF-Main.html
- **MeshLab (mesh editing):** https://www.meshlab.net/

---

## 💡 Pro Tips

1. **Keep meshes simple** - Complex meshes slow down RViz
2. **Use STL format** - Better compatibility than DAE
3. **Name everything clearly** - Makes URDF readable
4. **Test early** - View in RViz as soon as exported
5. **Version control** - Keep original SolidWorks + URDF in git

---

## ❓ Troubleshooting

**Q: OnShape doesn't have URDF export?**  
A: Check menu → Export → may be under "Download" options

**Q: Meshes are in wrong units?**  
A: Add scale factor in mesh tags (see Issue 1 above)

**Q: How do I set wheel speeds?**  
A: Will be done in ROS2 node (tank_control package)

**Q: Need to edit URDF after export?**  
A: Text editor works fine - structure is simple XML

---

**Once complete, you'll have a full robot model ready for ROS2 control!**

Let me know when you have the URDF extracted and ready to import!

