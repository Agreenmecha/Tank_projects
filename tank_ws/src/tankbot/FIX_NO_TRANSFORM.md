# Fix: "No Transform" Error for lidar_front

## Problem
Seeing "No transform from lidar_front to world" in RViz, even though `lidar_rear` works.

## Root Cause
`robot_state_publisher` is not running on your local machine. This node publishes all the URDF transforms (including `base_link`, `l_FL2`, `lidar_front`, etc.).

## Solution

Run the `remote_viz.launch.py` launch file on your **local machine**:

```bash
# On your local machine (laptop/desktop)
cd ~/Tank_projects/tank_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tankbot remote_viz.launch.py
```

This launch file will:
1. Start `robot_state_publisher` (publishes all URDF transforms)
2. Start `joint_state_publisher` (publishes joint states for continuous joints)
3. Start `static_transform_publisher` for `world->base_footprint`
4. Launch RViz2

## Verify It's Working

After launching, check that nodes are running:

```bash
ros2 node list
# Should see:
# /robot_state_publisher
# /simple_joint_state_publisher
# /world_to_base_footprint
# /rviz2
```

Check that transforms exist:

```bash
# Check base_link exists
ros2 run tf2_ros tf2_echo world base_link

# Check lidar_front transform
ros2 run tf2_ros tf2_echo world lidar_front

# Check lidar_rear transform
ros2 run tf2_ros tf2_echo world lidar_rear
```

All three should show transform data (not "frame does not exist" errors).

## Why lidar_rear "Worked"

If you saw `lidar_rear` working before, it was likely:
- Cached data in RViz from a previous session
- Or a different transform source

Both `lidar_front` and `lidar_rear` require the same TF tree:
```
world -> base_footprint -> base_link -> l_FL2/l_BL2 -> lidar_front/lidar_rear
```

Without `robot_state_publisher` running, none of these transforms exist.

