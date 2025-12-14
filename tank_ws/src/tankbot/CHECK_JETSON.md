# Check if Jetson Sensors Are Running

## Quick Check (Local Machine)

**Check if LiDAR topics exist:**
```bash
ros2 topic list | grep -i lidar
```

Should show:
- `/lidar_front/pointcloud`
- `/lidar_front/imu`
- `/lidar_rear/pointcloud`
- `/lidar_rear/imu`

**Check if topics have data:**
```bash
ros2 topic hz /lidar_front/pointcloud
ros2 topic hz /lidar_rear/pointcloud
```

Should show ~10 Hz for each.

**If topics are missing:**
The Jetson sensors are not running. You need to SSH to the Jetson and start them.

## Start Jetson Sensors

**SSH to Jetson:**
```bash
ssh aaronjet@<jetson_ip>
```

**On Jetson, start sensors:**
```bash
cd ~/Tank_projects/tank_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tank_sensors hardware.launch.py
```

## Verify Jetson Sensors Are Running

**On Jetson:**
```bash
ros2 node list | grep -i "lidar\|unitree"
```

Should show nodes like:
- `/lidar_front/lidar_front` (unitree_lidar_ros2_node)
- `/lidar_rear/lidar_rear` (unitree_lidar_ros2_node)

**Check topics on Jetson:**
```bash
ros2 topic list | grep -i lidar
```

## Network Check

**Verify ROS_DOMAIN_ID matches:**
Both machines must use the same ROS_DOMAIN_ID.

**On Jetson:**
```bash
echo $ROS_DOMAIN_ID
```

**On Local Machine:**
```bash
echo $ROS_DOMAIN_ID
```

Both should show the same number (e.g., `42`).

**If different, set them:**
```bash
# On both machines
export ROS_DOMAIN_ID=42
# Or add to ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
```

## Common Issues

1. **Topics not visible:** Jetson sensors not running or wrong ROS_DOMAIN_ID
2. **Topics exist but no data:** LiDARs not connected or wrong IP addresses
3. **Wrong frame_id:** Jetson launch file needs update (check LIDAR_FRAME_FIX.md)

