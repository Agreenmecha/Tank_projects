# Remote RViz Setup (Laptop → Jetson)

Run RViz on your laptop/desktop instead of the Jetson to save resources.

---

## Quick Setup

### On Jetson (Tank):

```bash
# 1. Get Jetson's IP
hostname -I

# 2. Set ROS_DOMAIN_ID (same on both machines)
export ROS_DOMAIN_ID=42

# 3. Launch without RViz
cd ~/Tank_projects/tank_ws
source install/setup.bash
ros2 launch tank_bringup full_autonomy.launch.py
```

### On Laptop/Desktop:

```bash
# 1. Install ROS2 Humble (if not already)
# Follow: https://docs.ros.org/en/humble/Installation.html

# 2. Set same ROS_DOMAIN_ID
export ROS_DOMAIN_ID=42

# 3. Source ROS2
source /opt/ros/humble/setup.bash

# 4. Install Nav2 (for RViz plugins)
sudo apt install ros-humble-nav2-bringup

# 5. Launch RViz with tank's config
rviz2 -d /path/to/nav2_default_view.rviz
```

---

## Option 1: Copy RViz Config to Laptop

```bash
# On laptop, copy config from Jetson
scp aaronjet@<jetson-ip>:~/Tank_projects/tank_ws/install/tank_navigation/share/tank_navigation/rviz/nav2_default_view.rviz ~/tank_rviz.rviz

# Launch RViz
rviz2 -d ~/tank_rviz.rviz
```

---

## Option 2: Use Default Nav2 Config

```bash
# On laptop
rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

Then manually add displays:
- **RobotModel** → `/robot_description`
- **PointCloud2** → `/lidar_front/cloud`
- **PointCloud2** → `/lidar_rear/cloud`
- **Odometry** → `/Odometry`
- **Map** → `/local_costmap/costmap`
- **Map** → `/global_costmap/costmap`
- **Path** → `/plan`

---

## Persistent ROS2 Network Setup

### On Both Machines (add to `~/.bashrc`):

```bash
# Add to ~/.bashrc
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# If on same network, ROS2 auto-discovers nodes
# No need for ROS_MASTER_URI like ROS1!
```

Then:
```bash
source ~/.bashrc
```

---

## Verify Connection

### On Laptop:

```bash
# Should see Jetson's nodes
ros2 node list

# Should see Jetson's topics
ros2 topic list

# Check data is flowing
ros2 topic hz /Odometry
ros2 topic hz /lidar_front/cloud
```

If topics visible → RViz will work! 🎉

---

## Troubleshooting

### Can't See Topics from Jetson

**Check network:**
```bash
# Ping Jetson from laptop
ping <jetson-ip>
```

**Check ROS_DOMAIN_ID matches:**
```bash
# On both machines
echo $ROS_DOMAIN_ID  # Should be same (e.g., 42)
```

**Check firewall (if needed):**
```bash
# On Jetson (if topics still not visible)
sudo ufw allow from <laptop-ip>
# Or disable for testing:
sudo ufw disable
```

**Force discovery (rarely needed):**
```bash
# On laptop, explicitly set Jetson's IP
export ROS_DISCOVERY_SERVER=<jetson-ip>:11811
```

### RViz Can't Load Robot Model

**Copy URDF to laptop:**
```bash
scp -r aaronjet@<jetson-ip>:~/Tank_projects/tank_ws/install/tank_description ~/tank_description_local
```

**Or set up NFS share** (advanced).

### High Latency / Lag

- Use wired Ethernet instead of WiFi
- Check network bandwidth: `iperf3 -s` (Jetson), `iperf3 -c <jetson-ip>` (laptop)
- Reduce point cloud density in costmap config

---

## Performance Tips

### Reduce Point Cloud Size (if laggy)

Edit `tank_ws/src/tank_navigation/config/nav2/nav2_params.yaml`:

```yaml
observation_sources: lidar_front lidar_rear
lidar_front:
  # Add downsampling
  voxel_size: 0.05  # 5cm voxels (reduces points)
```

### Use Compressed Transport (future)

Install on both machines:
```bash
sudo apt install ros-humble-image-transport-plugins
```

---

## SSH + RViz Workflow

### Recommended Setup:

**Terminal 1 (SSH to Jetson):**
```bash
ssh aaronjet@<jetson-ip>
cd ~/Tank_projects/tank_ws
source install/setup.bash
ros2 launch tank_bringup full_autonomy.launch.py
```

**Terminal 2 (Local laptop):**
```bash
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
rviz2 -d ~/tank_rviz.rviz
```

**Terminal 3 (Monitoring - either machine):**
```bash
ros2 topic hz /Odometry
ros2 topic list
rqt_graph
```

---

## Alternative: X11 Forwarding (Not Recommended)

If you really want to run RViz on Jetson but display on laptop:

```bash
ssh -X aaronjet@<jetson-ip>
rviz2
```

**Warning:** Much slower than native RViz on laptop.

---

## Checklist

On Jetson:
- [ ] `ROS_DOMAIN_ID=42` set
- [ ] `ROS_LOCALHOST_ONLY=0` set
- [ ] Full autonomy launched (without rviz)

On Laptop:
- [ ] ROS2 Humble installed
- [ ] `ROS_DOMAIN_ID=42` set (same as Jetson)
- [ ] `ROS_LOCALHOST_ONLY=0` set
- [ ] Can see Jetson's topics: `ros2 topic list`
- [ ] RViz launched with tank config

---

## Summary

**Jetson (robot):**
```bash
export ROS_DOMAIN_ID=42
ros2 launch tank_bringup full_autonomy.launch.py
```

**Laptop (visualization):**
```bash
export ROS_DOMAIN_ID=42
rviz2
```

That's it! ROS2's DDS automatically handles network discovery. 🚀

---

**Last Updated:** Dec 19, 2025  
**Status:** Ready for remote visualization

