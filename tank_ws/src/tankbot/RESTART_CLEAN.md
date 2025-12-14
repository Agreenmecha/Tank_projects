# Clean Restart Instructions for TF Issues

## Problem
Multiple `robot_state_publisher` instances were running, causing conflicts. `lidar_front` shows "no transform to world" even though `lidar_rear` works.

## Solution: Clean Restart

**Step 1: Kill all existing processes**

```bash
# Kill all robot state publisher and joint state publisher processes
pkill -f "robot_state_publisher|joint_state_publisher"

# Also kill RViz if running separately
pkill -f rviz2
```

**Step 2: Verify they're stopped**

```bash
ros2 node list
# Should show empty or only sensor nodes from Jetson
```

**Step 3: Restart visualization launch file**

```bash
cd ~/Tank_projects/tank_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tankbot remote_viz.launch.py
```

**Step 4: Verify nodes are running**

In a new terminal:
```bash
source /opt/ros/humble/setup.bash
source ~/Tank_projects/tank_ws/install/setup.bash
ros2 node list
```

Should see:
- `/robot_state_publisher`
- `/simple_joint_state_publisher`
- `/world_to_base_footprint`
- `/rviz2`

**Step 5: Verify transforms exist**

```bash
# Check base_link exists
ros2 run tf2_ros tf2_echo world base_link

# Check lidar_front transform
ros2 run tf2_ros tf2_echo world lidar_front

# Check lidar_rear transform  
ros2 run tf2_ros tf2_echo world lidar_rear
```

All three should show transform data.

**Step 6: Check TF static topic**

```bash
ros2 topic echo /tf_static --once | grep -E "(frame_id|child_frame_id)" | grep -i "lidar\|base"
```

Should show:
- `base_footprint -> base_link`
- `base_link -> l_FL2`
- `l_FL2 -> lidar_front`
- `base_link -> l_BL2`
- `l_BL2 -> lidar_rear`

## If lidar_front Still Doesn't Work

If `lidar_rear` works but `lidar_front` doesn't after clean restart:

1. **Check the URDF structure is correct:**
   ```bash
   check_urdf install/tankbot/share/tankbot/urdf/tankbot.urdf
   ```

2. **Verify l_FL2 joint exists:**
   ```bash
   grep -A 10 "j_FL2" src/tankbot/urdf/tankbot.urdf
   ```

3. **Rebuild the package:**
   ```bash
   cd ~/Tank_projects/tank_ws
   colcon build --packages-select tankbot
   source install/setup.bash
   ```

