# Remote Visualization: WSL, Windows, or Ubuntu Dual Boot

## Quick Answer

✅ **WSL2**: Yes, but requires X11 server setup  
✅ **Ubuntu Dual Boot**: Best option, easiest setup  
❌ **Windows Native**: No, can't run ROS2 directly

---

## Option 1: WSL2 (Windows Subsystem for Linux)

### Pros:
- No need to reboot
- Can run alongside Windows
- Full Linux environment

### Cons:
- Requires X11 server setup (WSLg or VcXsrv)
- Slightly more complex setup
- Performance may be slightly slower

### Setup Steps:

#### Step 1: Install ROS2 Humble in WSL2

```bash
# In WSL2 terminal
sudo apt update
sudo apt install ros-humble-desktop

# Add to ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
source ~/.bashrc
```

#### Step 2A: Use WSLg (Windows 11, recommended)

**Windows 11** has built-in WSLg support:
- Just install WSL2 with Ubuntu
- GUI apps work automatically!
- No additional setup needed

```bash
# In WSL2, just run:
rviz2
```

#### Step 2B: Use VcXsrv (Windows 10 or if WSLg doesn't work)

1. **Install VcXsrv on Windows:**
   - Download: https://sourceforge.net/projects/vcxsrv/
   - Install and launch "XLaunch"
   - Settings:
     - Display number: `0`
     - Start no client: ✓
     - Disable access control: ✓
     - Clipboard: ✓
     - Primary Selection: ✓

2. **Configure WSL2 to use VcXsrv:**
   ```bash
   # In WSL2, add to ~/.bashrc
   export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0.0
   export LIBGL_ALWAYS_INDIRECT=1
   
   # Or get Windows IP dynamically:
   export DISPLAY=$(ip route | grep default | awk '{print $3}'):0.0
   ```

3. **Test X11:**
   ```bash
   # In WSL2
   xeyes  # Should open a window
   # Or
   xclock
   ```

#### Step 3: Connect to Jetson ROS2

```bash
# In WSL2, verify network connectivity
ping 192.168.2.100

# Check ROS2 connection
ros2 topic list
# Should show Jetson topics

# Launch RViz
rviz2
```

---

## Option 2: Ubuntu Dual Boot (Recommended)

### Pros:
- Native Linux performance
- Easiest setup
- Best compatibility
- No X11 forwarding issues

### Cons:
- Need to reboot to switch OS
- Requires Ubuntu installation

### Setup Steps:

1. **Boot into Ubuntu**

2. **Install ROS2 Humble:**
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop
   ```

3. **Configure ROS2:**
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
   echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
   source ~/.bashrc
   ```

4. **Connect to Jetson:**
   ```bash
   # Verify network (may need to connect to same WiFi)
   ping 192.168.2.100
   
   # Check ROS2 topics
   ros2 topic list
   
   # Launch RViz
   rviz2
   ```

---

## Option 3: Windows Native (Not Recommended)

**Cannot run ROS2 directly on Windows.**

### Workaround: Use Remote Desktop/VNC

1. **On Jetson, install VNC server:**
   ```bash
   sudo apt install tigervnc-standalone-server
   vncserver :1
   ```

2. **On Windows, use VNC viewer:**
   - Download: https://www.realvnc.com/en/connect/download/viewer/
   - Connect to: `192.168.2.100:5901`
   - Run RViz in the VNC session

**Pros:** Works from Windows  
**Cons:** Higher latency, requires desktop environment on Jetson

---

## Network Configuration

### Important: Laptop must reach Jetson

**Jetson IP:** `192.168.2.100` (lidar network)

**If Jetson is on WiFi:**
- Connect laptop to same WiFi network
- Find Jetson WiFi IP: `ssh aaronjet@192.168.2.100 'hostname -I'`

**If Jetson is only on lidar network:**
- Laptop needs route to `192.168.2.0/24`
- Or connect laptop to WiFi that bridges to lidar network

### Test Connectivity

```bash
# From laptop (WSL2 or Ubuntu)
ping 192.168.2.100

# Should get responses
```

---

## Recommended Setup by Scenario

### Scenario 1: Windows 11 with WSL2
→ **Use WSLg** (built-in, easiest)

### Scenario 2: Windows 10 with WSL2
→ **Use VcXsrv** for X11 forwarding

### Scenario 3: Ubuntu Dual Boot Available
→ **Use Ubuntu** (best performance)

### Scenario 4: Only Windows, No WSL
→ **Use VNC** to Jetson (remote desktop)

---

## Quick Test Commands

**In WSL2 or Ubuntu:**

```bash
# 1. Check ROS2 installed
ros2 --help

# 2. Check ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID  # Should be: 42

# 3. Test network
ping 192.168.2.100

# 4. Test ROS2 connection
ros2 topic list
# Should show: /lidar_front/pointcloud, /lidar_rear/pointcloud, etc.

# 5. Test topic rate
ros2 topic hz /lidar_front/pointcloud
# Should show: ~10 Hz

# 6. Launch RViz
rviz2
```

---

## Troubleshooting

### WSL2: "Cannot connect to X server"

**Solution:**
```bash
# Check DISPLAY variable
echo $DISPLAY

# Set DISPLAY (for VcXsrv)
export DISPLAY=$(ip route | grep default | awk '{print $3}'):0.0

# Or for WSLg (Windows 11)
export DISPLAY=:0
```

### WSL2: "No topics visible"

**Check:**
1. Same ROS_DOMAIN_ID? (`echo $ROS_DOMAIN_ID`)
2. Network connectivity? (`ping 192.168.2.100`)
3. Firewall blocking? (Check Windows Firewall)

### Ubuntu: "No topics visible"

**Check:**
1. Same ROS_DOMAIN_ID
2. Network connectivity
3. Restart ROS2 daemon: `ros2 daemon stop && ros2 daemon start`

---

## Performance Comparison

| Method | Latency | Setup Difficulty | GPU Usage |
|--------|---------|------------------|-----------|
| Ubuntu Dual Boot | Low | Easy | Laptop GPU |
| WSL2 + WSLg | Low-Medium | Easy | Laptop GPU |
| WSL2 + VcXsrv | Medium | Medium | Laptop GPU |
| VNC to Jetson | High | Medium | Jetson GPU |

---

## My Recommendation

**If you have Ubuntu dual boot:** Use it (easiest, best performance)

**If only Windows:**
- **Windows 11:** Use WSL2 with WSLg
- **Windows 10:** Use WSL2 with VcXsrv

---

**Last Updated:** 2025-01-27

