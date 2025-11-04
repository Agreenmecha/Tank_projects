# Remote ROS 2 Visualization - Jetson to Desktop

**Goal:** Run RViz2 on your desktop computer to visualize data from the Jetson

---

## Quick Start

### 1. On Jetson (Configure Once)

```bash
# Set ROS_DOMAIN_ID
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc

# Launch LiDARs
cd ~/Tank_projects/tank_ws
source install/setup.bash
ros2 launch tank_sensors dual_lidar.launch.py
```

### 2. On Desktop

```bash
# Install ROS 2 Humble (if not already)
sudo apt update
sudo apt install ros-humble-desktop

# Set same ROS_DOMAIN_ID
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc

# Launch RViz
rviz2
```

### 3. In RViz

1. **Fixed Frame**: Set to `lidar_front`
2. **Add Displays**:
   - Click "Add" → "By topic"
   - Select `/lidar_front/pointcloud` → PointCloud2
   - Select `/lidar_rear/pointcloud` → PointCloud2
3. **Adjust Colors**: Change point cloud colors for differentiation

---

## Network Requirements

### Same Network
- ✅ Jetson: `192.168.1.167` (WiFi)
- ✅ Desktop: Must be on `192.168.1.0/24` subnet
- ✅ Both can ping each other

Test connectivity:
```bash
# From Desktop
ping 192.168.1.167

# From Jetson  
ping <your_desktop_ip>
```

### ROS_DOMAIN_ID
Both machines **must** have the **same** `ROS_DOMAIN_ID` (0-101).

**Why?** ROS 2 uses DDS multicast for discovery. Same domain = can see each other.

```bash
# On BOTH machines
export ROS_DOMAIN_ID=42
```

---

## Detailed Setup

### Jetson Configuration

**File: `~/.bashrc`** (add these lines):
```bash
# ROS 2 Humble
source /opt/ros/humble/setup.bash

# Tank workspace
source ~/Tank_projects/tank_ws/install/setup.bash

# Network configuration
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# ODrive tools
export PATH=$PATH:/home/aaronjet/.local/bin
```

Apply changes:
```bash
source ~/.bashrc
```

### Desktop Configuration

**File: `~/.bashrc`** (add these lines):
```bash
# ROS 2 Humble
source /opt/ros/humble/setup.bash

# Network configuration (MUST match Jetson!)
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Apply changes:
```bash
source ~/.bashrc
```

---

## Verification

### On Desktop (with Jetson running)

**Check topic discovery:**
```bash
ros2 topic list
```

**Expected output:**
```
/lidar_front/pointcloud
/lidar_front/imu
/lidar_rear/pointcloud
/lidar_rear/imu
/parameter_events
/rosout
/tf
```

**Check topic data rate:**
```bash
ros2 topic hz /lidar_front/pointcloud
# Should show: ~10 Hz
```

**Echo data:**
```bash
ros2 topic echo /lidar_front/pointcloud --once
```

---

## Troubleshooting

### Issue: No Topics Visible on Desktop

**Check 1: Same ROS_DOMAIN_ID?**
```bash
# On Jetson
echo $ROS_DOMAIN_ID

# On Desktop
echo $ROS_DOMAIN_ID

# Should be identical!
```

**Check 2: Network connectivity**
```bash
# From Desktop
ping 192.168.1.167
```

**Check 3: Firewall blocking multicast?**
```bash
# On Jetson
sudo ufw status

# If active, allow subnet:
sudo ufw allow from 192.168.1.0/24

# On Desktop (if firewall active)
sudo ufw allow from 192.168.1.0/24
```

**Check 4: ROS 2 running on Jetson?**
```bash
# On Jetson
ros2 node list
# Should show: /lidar_front, /lidar_rear
```

### Issue: Topics Visible but No Data in RViz

**Check Fixed Frame:**
- In RViz, Global Options → Fixed Frame
- Set to: `lidar_front` or `lidar_rear`
- NOT `map` or `base_link` (those don't exist yet)

**Check Point Cloud Display:**
- Size (m): Try `0.01` to `0.05`
- Style: Try `Flat Squares` or `Points`
- Color: Try `Intensity` or `Flat Color`

### Issue: RViz Crashes or Lags

**Use lighter visualization:**
```bash
# Launch with reduced point density
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=false
```

**Or filter point clouds:**
- In RViz, PointCloud2 display
- Reduce "Decay Time"
- Increase "Selectable" to false

---

## Advanced: Custom RViz Configuration

### Save Configuration

1. In RViz, configure your displays
2. File → Save Config As → `~/tank_lidar.rviz`

### Load Configuration

```bash
rviz2 -d ~/tank_lidar.rviz
```

### Create Desktop Launch File

**File: `~/start_tank_viz.sh`**
```bash
#!/bin/bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
rviz2 -d ~/tank_lidar.rviz
```

```bash
chmod +x ~/start_tank_viz.sh
./start_tank_viz.sh
```

---

## RViz Configuration Tips

### Recommended Display Settings

**PointCloud2 - Front LiDAR:**
- Topic: `/lidar_front/pointcloud`
- Size (m): `0.02`
- Style: `Flat Squares`
- Color Transformer: `Intensity`
- Channel Name: `intensity`
- Min Value: `0`
- Max Value: `255`

**PointCloud2 - Rear LiDAR:**
- Topic: `/lidar_rear/pointcloud`  
- Size (m): `0.02`
- Style: `Flat Squares`
- Color Transformer: `Flat Color`
- Color: `Red` (to differentiate from front)

**Grid:**
- Add → Grid
- Reference Frame: `lidar_front`
- Cell Size: `1.0`

**TF:**
- Add → TF
- Shows coordinate frame relationships

---

## Network Architecture

```
┌─────────────────┐         WiFi         ┌─────────────────┐
│   Desktop PC    │   192.168.1.0/24    │  Jetson Orin   │
│                 │◄────────────────────►│                 │
│  RViz2 Client   │   ROS_DOMAIN_ID=42   │  LiDAR Nodes   │
│  (Visualizer)   │                      │  (Publishers)  │
└─────────────────┘                      └─────────────────┘
                                                  │
                                         ┌────────┴────────┐
                                         │                 │
                                    LiDAR Front      LiDAR Rear
                                    192.168.2.62    192.168.2.63
```

---

## Performance Considerations

### Bandwidth Usage
- Each LiDAR: ~2500 points × 10 Hz = ~1-2 Mbps
- Both LiDARs: ~2-4 Mbps
- WiFi sufficient for visualization

### Desktop Requirements
- Ubuntu 22.04
- ROS 2 Humble
- OpenGL 3.3+ (for RViz)
- 4GB+ RAM recommended

### Reduce Network Load (Optional)

**Option 1: Reduce point cloud rate**
Modify launch file `cloud_scan_num` parameter:
```python
{'cloud_scan_num': 36}  # Half the points
```

**Option 2: Compress point clouds**
```bash
# On Jetson
sudo apt install ros-humble-compressed-image-transport

# TODO: Implement compression in future
```

---

## Alternative: VNC/Remote Desktop

If ROS 2 network discovery doesn't work, use VNC:

### On Jetson

```bash
# Install VNC server
sudo apt install tigervnc-standalone-server

# Start VNC
vncserver :1

# Kill VNC
vncserver -kill :1
```

### On Desktop

```bash
# Install VNC viewer
sudo apt install tigervnc-viewer

# Connect to Jetson
vncviewer 192.168.1.167:5901
```

Then run RViz directly on Jetson (forwarded to desktop).

**Pros:** Always works  
**Cons:** Higher latency, requires desktop environment on Jetson

---

## Testing Checklist

On Desktop, after setup:

- [ ] `echo $ROS_DOMAIN_ID` shows `42`
- [ ] `ping 192.168.1.167` successful
- [ ] `ros2 topic list` shows LiDAR topics
- [ ] `ros2 topic hz /lidar_front/pointcloud` shows ~10 Hz
- [ ] RViz opens without errors
- [ ] Point clouds visible in RViz
- [ ] Point clouds update in real-time

---

## Useful Commands Reference

```bash
# Check ROS 2 network
ros2 daemon stop && ros2 daemon start

# List all nodes (from any machine)
ros2 node list

# See node information
ros2 node info /lidar_front

# Monitor topic bandwidth
ros2 topic bw /lidar_front/pointcloud

# Check message type
ros2 topic type /lidar_front/pointcloud

# Echo topic (minimal)
ros2 topic echo /lidar_front/pointcloud --once --no-arr

# Record data
ros2 bag record /lidar_front/pointcloud /lidar_rear/pointcloud

# Play back recorded data
ros2 bag play <bag_file>
```

---

## Next Steps

Once visualization is working:

1. ✅ Verify both LiDAR point clouds
2. ✅ Check IMU data is streaming
3. [ ] Add static transforms between lidars
4. [ ] Integrate Point-LIO for localization
5. [ ] Visualize odometry in RViz

---

**Status:** Ready for remote visualization  
**Updated:** 2025-11-04

