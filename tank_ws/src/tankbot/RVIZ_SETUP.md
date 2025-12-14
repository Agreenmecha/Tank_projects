# RViz Setup Instructions

If you're not seeing the robot model in RViz, follow these steps:

## Step 1: Launch the display
```bash
cd ~/Tank_projects/tank_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tankbot display.launch.py
```

## Step 2: Configure RViz Displays

1. **Set Fixed Frame:**
   - In the "Global Options" section at the top of the Displays panel
   - Change `Fixed Frame` from `map` to `world` (or `base_link` if world doesn't work)

2. **Add RobotModel Display:**
   - Click "Add" button at the bottom of Displays panel
   - Select "RobotModel" from the list
   - Expand the RobotModel settings:
     - **Description Source:** Select "Topic"
     - **Description Topic:** Should be `/robot_description`
     - **Visual Enabled:** ✓ Checked
     - **Collision Enabled:** Can be unchecked for visualization
   
3. **Camera View:**
   - The robot should appear at the origin (0, 0, 0)
   - Use "Move Camera" tool (hand icon) to zoom in/out
   - Use "Focus Camera" (crosshair icon) and click on the robot to center view
   - Try clicking "Reset" in the Views panel

## Step 3: Troubleshooting

If still not visible:

1. **Check TF tree:**
   ```bash
   ros2 run tf2_tools view_frames
   ```
   This creates a `frames.pdf` showing the TF tree

2. **Verify transforms are published:**
   ```bash
   ros2 topic echo /tf --once
   ```
   Should show transforms from base_link to all child links

3. **Check robot description:**
   ```bash
   ros2 topic echo /robot_description --once
   ```
   Should show the full URDF XML

4. **Verify meshes exist:**
   ```bash
   ls -la install/tankbot/share/tankbot/meshes/*.STL
   ```

## Common Issues

- **Robot appears but is tiny:** The meshes might be in millimeters instead of meters. Check the URDF scale or mesh units.
- **Robot appears but is huge:** Same issue, scale problem.
- **No TF transforms:** Make sure `joint_state_publisher` is running (check with `ros2 node list`)
- **Wrong frame:** Make sure Fixed Frame matches a frame in the TF tree (try `world` or `base_link`)

