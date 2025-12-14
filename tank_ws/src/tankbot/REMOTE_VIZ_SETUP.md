# Remote Visualization Setup - Tankbot with LiDAR Data

Visualize the rover on your local machine with live LiDAR data from Jetson Orin Nano.

---

## Quick Start

### Step 1: Configure ROS2 Network (Both Machines)

**On Jetson:**
```bash
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
source ~/.bashrc
```

**On Local Machine:**
```bash
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
source ~/.bashrc
```

**Important:** Both machines **must** use the same `ROS_DOMAIN_ID` (42 in this case).

### Step 2: Launch Sensors on Jetson

```bash
# SSH to Jetson
ssh aaronjet@<jetson_ip>

# On Jetson
cd ~/Tank_projects/tank_ws
source install/setup.bash
ros2 launch tank_sensors hardware.launch.py
```

This launches:
- Dual LiDARs (front + rear)
- GNSS
- Robot state publisher with URDF

### Step 3: Launch Visualization on Local Machine

```bash
# On your local machine
cd ~/Tank_projects/tank_ws
source install/setup.bash
ros2 launch tankbot remote_viz.launch.py
```

---

## Verify Connection

### Check Topics Are Visible

On local machine:
```bash
ros2 topic list
```

You should see:
```
/lidar_front/pointcloud
/lidar_rear/pointcloud
/lidar_front/imu
/lidar_rear/imu
/robot_description
/tf
/tf_static
/gnss/fix
...
```

### Check Topic Data Rate

```bash
ros2 topic hz /lidar_front/pointcloud
# Should show: ~10 Hz
```

---

## RViz Configuration

### Initial Setup

1. **Set Fixed Frame:**
   - Global Options → Fixed Frame: `world` or `base_footprint`

2. **Add RobotModel:**
   - Click "Add" → "By display type" → "RobotModel"
   - Description Source: "Topic"
   - Description Topic: `/robot_description`
   - Visual Enabled: ✓

3. **Add Front LiDAR PointCloud:**
   - Click "Add" → "By topic"
   - Select `/lidar_front/pointcloud` → "PointCloud2"
   - Name it: "Front LiDAR"
   - Settings:
     - Size (m): `0.02`
     - Style: `Flat Squares`
     - Color Transformer: `Intensity`
     - Min Value: `0`, Max Value: `255`

4. **Add Rear LiDAR PointCloud:**
   - Click "Add" → "By topic"
   - Select `/lidar_rear/pointcloud` → "PointCloud2"
   - Name it: "Rear LiDAR"
   - Settings:
     - Size (m): `0.02`
     - Style: `Flat Squares`
     - Color Transformer: `Flat Color`
     - Color: `Red` (to differentiate from front)

5. **Add TF Display (Optional):**
   - Click "Add" → "By display type" → "TF"
   - Shows all coordinate frames

---

## Network Troubleshooting

### Issue: No Topics Visible

**Check 1: Same ROS_DOMAIN_ID?**
```bash
# On Jetson
echo $ROS_DOMAIN_ID  # Should show: 42

# On Local Machine
echo $ROS_DOMAIN_ID  # Should show: 42
```

**Check 2: Network Connectivity**
```bash
# From local machine
ping <jetson_ip>

# From Jetson
ping <local_machine_ip>
```

**Check 3: Firewall**
```bash
# On Jetson
sudo ufw status
# If active, allow your subnet:
sudo ufw allow from <local_network>/24

# On Local Machine (if firewall active)
sudo ufw allow from <jetson_network>/24
```

**Check 4: ROS2 Running on Jetson?**
```bash
# On Jetson
ros2 node list
# Should show sensor nodes
```

### Issue: Transforms Not Working

**Verify robot_description is published:**
```bash
ros2 topic echo /robot_description --once
```

**Check TF tree:**
```bash
ros2 run tf2_tools view_frames
# Opens frames.pdf showing transform tree
```

---

## Network Architecture

```
┌─────────────────────┐         Same Network          ┌─────────────────────┐
│   Local Machine     │    ROS_DOMAIN_ID=42          │  Jetson Orin Nano  │
│                     │◄─────────────────────────────►│                     │
│  RViz2              │    (WiFi/Ethernet)           │  Sensor Nodes      │
│  robot_state_pub    │                               │  LiDAR Drivers     │
│  (subscribes to     │                               │  GNSS Driver       │
│   sensor topics)    │                               │  (publishes topics)│
└─────────────────────┘                               └─────────────────────┘
                                                               │
                                                    ┌──────────┴──────────┐
                                                    │                     │
                                              Front LiDAR         Rear LiDAR
                                              192.168.123.123   192.168.123.124
```

---

## Save RViz Configuration

Once configured, save for future use:

1. In RViz: **File → Save Config As**
2. Save to: `~/tankbot_remote_viz.rviz`
3. Next time: `rviz2 -d ~/tankbot_remote_viz.rviz`

---

## Performance Tips

- **Point Cloud Decay Time:** Reduce to `5-10` seconds to limit memory usage
- **Update Rate:** Point clouds update at ~10 Hz (LiDAR rate)
- **Network Bandwidth:** Each LiDAR ~1-2 Mbps, total ~2-4 Mbps

---

## What You'll See

- **Robot Model:** Tankbot URDF model with all links and joints
- **Front LiDAR Point Cloud:** Colored by intensity, showing forward scan
- **Rear LiDAR Point Cloud:** Red colored, showing rear scan
- **TF Frames:** All sensor frames positioned correctly on robot
- **GNSS Position:** If GNSS is enabled, fix data available

