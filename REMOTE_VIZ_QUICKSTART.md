# Remote Visualization Quick Start

**Current Setup:**
- Jetson IP: `192.168.2.100` (lidar network)
- ROS_DOMAIN_ID: `42`
- LiDAR nodes: Running ✅
- GNSS node: Running ✅

---

## Option 1: X11 Forwarding (RViz on Jetson, Display on Laptop)

**Best for:** Using Jetson GPU for rendering

### On Your Laptop (SSH with X11 forwarding):

```bash
# Connect with X11 forwarding enabled
ssh -X aaronjet@192.168.2.100

# Or if using a different network interface:
ssh -X aaronjet@<jetson_wifi_ip>
```

### On Jetson (after SSH):

```bash
cd ~/Tank_projects
./scripts/start_rviz_remote.sh
```

**Or manually:**
```bash
cd ~/Tank_projects/tank_ws
source install/setup.bash
rviz2
```

### In RViz2:
1. **Fixed Frame**: `lidar_front`
2. **Add Displays**:
   - Click "Add" → "By topic"
   - `/lidar_front/pointcloud` → PointCloud2
   - `/lidar_rear/pointcloud` → PointCloud2
3. **Point Cloud Settings**:
   - Size (m): `0.02`
   - Style: `Flat Squares`
   - Color Transformer: `Intensity` (for front)
   - Color: `Red` (for rear, to differentiate)

---

## Option 2: Distributed ROS2 (RViz on Laptop)

**Best for:** Better performance, uses laptop GPU

### Step 1: On Your Laptop

```bash
# Install ROS2 Humble (if not already installed)
sudo apt update
sudo apt install ros-humble-desktop

# Set ROS_DOMAIN_ID to match Jetson
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
source ~/.bashrc

# Verify
echo $ROS_DOMAIN_ID  # Should show: 42
```

### Step 2: Network Connection

**Important:** Your laptop must be on the same network as the Jetson.

**If Jetson is on WiFi:**
- Connect laptop to same WiFi network
- Find Jetson WiFi IP: `ssh aaronjet@192.168.2.100 'hostname -I'`

**If Jetson is on lidar network (192.168.2.100):**
- Laptop needs route to lidar network OR
- Connect laptop to WiFi that has access to lidar network

### Step 3: Verify Connection

```bash
# On laptop, test connectivity
ping 192.168.2.100

# Check if you can see Jetson topics
ros2 topic list
# Should show: /lidar_front/pointcloud, /lidar_rear/pointcloud, etc.

# Check topic rate
ros2 topic hz /lidar_front/pointcloud
# Should show: ~10 Hz
```

### Step 4: Launch RViz on Laptop

```bash
rviz2
```

**In RViz2:**
- Fixed Frame: `lidar_front`
- Add PointCloud2 displays for both LiDARs

---

## Troubleshooting

### X11 Forwarding Issues

**Problem:** `DISPLAY not set` or `cannot connect to X server`

**Solution:**
```bash
# On laptop, enable X11 forwarding
xhost +local:

# Reconnect with X11
ssh -X aaronjet@192.168.2.100

# On Jetson, check display
echo $DISPLAY  # Should show something like localhost:10.0
```

### ROS2 Topics Not Visible

**Problem:** `ros2 topic list` on laptop doesn't show Jetson topics

**Check 1: Same ROS_DOMAIN_ID?**
```bash
# On Jetson
echo $ROS_DOMAIN_ID  # Should be: 42

# On Laptop
echo $ROS_DOMAIN_ID  # Should be: 42
```

**Check 2: Network connectivity**
```bash
# From laptop
ping 192.168.2.100
```

**Check 3: Firewall**
```bash
# On Jetson
sudo ufw status
# If active, allow ROS2 multicast:
sudo ufw allow from <laptop_ip>
```

**Check 4: Restart ROS2 daemon**
```bash
# On both machines
ros2 daemon stop
ros2 daemon start
```

### RViz Shows No Data

**Check Fixed Frame:**
- Must be set to `lidar_front` or `lidar_rear`
- NOT `map` or `base_link` (those don't exist yet)

**Check Topic Names:**
- Verify topics exist: `ros2 topic list`
- Check topic is publishing: `ros2 topic hz /lidar_front/pointcloud`

---

## Current Status

✅ **Jetson Configuration:**
- ROS_DOMAIN_ID: 42
- LiDAR nodes: Running
- GNSS node: Running
- Topics available: 40

✅ **Ready for Visualization**

---

## Quick Commands

**On Jetson:**
```bash
# Check nodes
ros2 node list

# Check topics
ros2 topic list

# Check topic rate
ros2 topic hz /lidar_front/pointcloud

# Start RViz (X11 forwarding)
cd ~/Tank_projects && ./scripts/start_rviz_remote.sh
```

**On Laptop:**
```bash
# Verify connection
ros2 topic list

# Start RViz
rviz2
```

---

**Last Updated:** $(date)

