# Start Jetson Sensors

## Status: LiDAR Topics Not Visible

The LiDAR topics (`/lidar_front/pointcloud`, `/lidar_rear/pointcloud`) are not visible on your local machine. This means the Jetson sensors are not running or not reachable.

## Step 1: Verify Network Connection

**On local machine, check if you can reach Jetson:**
```bash
ping <jetson_ip>
# Replace <jetson_ip> with your Jetson's IP address
```

## Step 2: Check ROS_DOMAIN_ID

**Both machines must use the same ROS_DOMAIN_ID.**

**On local machine:**
```bash
echo $ROS_DOMAIN_ID
```

**Expected:** Should show a number (e.g., `42`). If empty or wrong, set it:
```bash
export ROS_DOMAIN_ID=42
# Or add to ~/.bashrc for permanent:
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc
```

## Step 3: SSH to Jetson

```bash
ssh aaronjet@<jetson_ip>
# Replace <jetson_ip> with your Jetson's IP address
```

## Step 4: On Jetson - Check Current State

**Check if sensors are already running:**
```bash
ros2 node list | grep -i "lidar\|unitree"
```

If you see nodes, they're running. If empty, proceed to start them.

**Check ROS_DOMAIN_ID on Jetson:**
```bash
echo $ROS_DOMAIN_ID
```

Should match your local machine (e.g., `42`). If not, set it:
```bash
export ROS_DOMAIN_ID=42
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc
```

## Step 5: On Jetson - Start Sensors

```bash
cd ~/Tank_projects/tank_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tank_sensors hardware.launch.py
```

**Expected output:**
- Should see LiDAR nodes starting
- Should see connection messages for LiDARs
- Should see GNSS node starting (if enabled)

**Keep this terminal open** - the sensors need to keep running.

## Step 6: Verify on Local Machine

**In a new terminal on local machine:**
```bash
cd ~/Tank_projects/tank_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Check topics are now visible
ros2 topic list | grep -i lidar

# Should see:
# /lidar_front/pointcloud
# /lidar_front/imu
# /lidar_rear/pointcloud
# /lidar_rear/imu

# Check data rate
ros2 topic hz /lidar_front/pointcloud
# Should show ~10 Hz
```

## Troubleshooting

### Topics still not visible:

1. **Check ROS_DOMAIN_ID matches** on both machines
2. **Check firewall** - ports should be open for ROS2 communication
3. **Check network** - both machines on same network
4. **Restart ROS on both machines** after changing ROS_DOMAIN_ID

### LiDAR connection errors on Jetson:

1. **Check LiDAR IP addresses** in `lidar_dual.launch.py`:
   - Front: `192.168.123.123`
   - Rear: `192.168.123.124`

2. **Check network connection** on Jetson:
   ```bash
   ping 192.168.123.123
   ping 192.168.123.124
   ```

3. **Check LiDAR power** - ensure both LiDARs are powered on

### Frame ID issues:

If topics appear but have wrong frame_id, see `LIDAR_FRAME_FIX.md` for instructions to rebuild and restart.

