# Dual Unitree L2 LiDAR Setup - COMPLETE ✅

**Date Completed:** 2025-11-04  
**System:** Jetson Orin Nano + 2x Unitree L2 LiDARs + Network Switch

---

## Final Network Configuration

```
                    Network Switch (192.168.2.0/24)
                          |
        +-----------------+------------------+
        |                 |                  |
   Jetson Orin      LiDAR Front        LiDAR Rear
  192.168.2.100    192.168.2.62      192.168.2.63
   eno1 interface   Port: 6201        Port: 6202
```

### IP Address Table

| Device | IP Address | Send Port | Target IP | Target Port | Status |
|--------|------------|-----------|-----------|-------------|--------|
| **Jetson (eno1)** | 192.168.2.100 | - | - | 6201, 6202 | ✅ Active |
| **LiDAR Front** | 192.168.2.62 | 6101 | 192.168.2.100 | 6201 | ✅ Tested |
| **LiDAR Rear** | 192.168.2.63 | 6101 | 192.168.2.100 | 6202 | ✅ Tested |
| **Gateway** | 192.168.2.1 | - | - | - | Configured |
| **Subnet Mask** | 255.255.255.0 | - | - | - | All devices |

**Note:** LiDAR network is on separate subnet (192.168.2.x) from WiFi (192.168.1.x) to avoid routing conflicts.

---

## Verification Results

### Test Date: 2025-11-04 23:10 UTC

**Front LiDAR (192.168.2.62):**
```bash
$ ping -c 5 192.168.2.62
5 packets transmitted, 5 received, 0% packet loss
rtt min/avg/max/mdev = 0.149/0.211/0.292/0.053 ms

$ sudo tcpdump -i eno1 -n udp port 6201 -c 5
673 packets received by filter
✅ Point cloud data streaming successfully
```

**Rear LiDAR (192.168.2.63):**
```bash
$ ping -c 5 192.168.2.63
5 packets transmitted, 5 received, 0% packet loss
rtt min/avg/max/mdev = 0.227/0.366/0.523/0.102 ms

$ sudo tcpdump -i eno1 -n udp port 6202 -c 5
680 packets received by filter
✅ Point cloud data streaming successfully
```

**Both LiDARs can stream simultaneously without interference.**

---

## Quick Test Commands

### Check Network Interface
```bash
ip addr show eno1
# Should show: inet 192.168.2.100/24
```

### Test LiDAR Connectivity
```bash
# Front
ping -c 3 192.168.2.62

# Rear
ping -c 3 192.168.2.63
```

### Monitor Point Cloud Data
```bash
# Front LiDAR data
sudo tcpdump -i eno1 -n udp port 6201 -c 10

# Rear LiDAR data
sudo tcpdump -i eno1 -n udp port 6202 -c 10

# Both simultaneously
sudo tcpdump -i eno1 -n "udp port 6201 or udp port 6202" -c 20
```

### Check Active Connections
```bash
# Show LiDAR traffic
sudo tcpdump -i eno1 net 192.168.2.0/24 -n

# Show UDP listeners
sudo ss -ulnp | grep -E "6201|6202"
```

---

## LiDAR Configuration Details

### Front LiDAR Settings (via UART)
```
Lidar:
  ChangeIP:     192.168.2.62
  ChangePort:   6101
  Gateway:      192.168.2.1
  SubnetMask:   255.255.255.0

Usr:
  ChangeIP:     192.168.2.100  (Jetson)
  ChangePort:   6201

Work Mode:
  ENET/UART:    ENET
  Normal/Standby: Normal
```

### Rear LiDAR Settings (via UART)
```
Lidar:
  ChangeIP:     192.168.2.63
  ChangePort:   6101
  Gateway:      192.168.2.1
  SubnetMask:   255.255.255.0

Usr:
  ChangeIP:     192.168.2.100  (Jetson)
  ChangePort:   6202

Work Mode:
  ENET/UART:    ENET
  Normal/Standby: Normal
```

---

## Jetson Network Configuration

**NetworkManager Connection:**
```bash
Connection name: lidar-network
Interface: eno1
Method: manual
IP: 192.168.2.100/24
Autoconnect: yes
```

**Configuration command:**
```bash
sudo nmcli connection add \
    type ethernet \
    con-name lidar-network \
    ifname eno1 \
    ipv4.method manual \
    ipv4.addresses 192.168.2.100/24 \
    autoconnect yes

sudo nmcli connection up lidar-network
```

**To restart network:**
```bash
sudo nmcli connection down lidar-network
sudo nmcli connection up lidar-network
```

---

## Next Steps for ROS 2 Integration

### 1. Install Unitree SDK
```bash
cd ~/Tank_projects/tank_ws/src/external
git clone https://github.com/unitreerobotics/unilidar_sdk2.git

cd ~/Tank_projects/tank_ws
colcon build --packages-select unilidar_sdk2
source install/setup.bash
```

### 2. Create tank_sensors Package
```bash
cd ~/Tank_projects/tank_ws/src
ros2 pkg create tank_sensors \
    --build-type ament_cmake \
    --dependencies rclcpp sensor_msgs
```

### 3. Launch File Configuration

**File:** `tank_ws/src/tank_sensors/launch/lidar.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Front LiDAR
        Node(
            package='unilidar_sdk2',
            executable='unilidar_sdk2_node',
            name='lidar_front',
            parameters=[{
                'lidar_ip': '192.168.2.62',
                'lidar_port': 6101,
                'local_port': 6201,
                'frame_id': 'lidar_front'
            }],
            remappings=[
                ('cloud', '/lidar_front/pointcloud')
            ]
        ),
        
        # Rear LiDAR
        Node(
            package='unilidar_sdk2',
            executable='unilidar_sdk2_node',
            name='lidar_rear',
            parameters=[{
                'lidar_ip': '192.168.2.63',
                'lidar_port': 6101,
                'local_port': 6202,
                'frame_id': 'lidar_rear'
            }],
            remappings=[
                ('cloud', '/lidar_rear/pointcloud')
            ]
        )
    ])
```

### 4. Test ROS 2 Point Clouds
```bash
# Launch LiDARs
ros2 launch tank_sensors lidar.launch.py

# In another terminal, verify topics
ros2 topic list | grep lidar

# View point cloud data
ros2 topic echo /lidar_front/pointcloud --once
ros2 topic echo /lidar_rear/pointcloud --once

# Check data rate
ros2 topic hz /lidar_front/pointcloud
ros2 topic hz /lidar_rear/pointcloud
```

---

## Troubleshooting

### Issue: Cannot ping LiDAR
```bash
# Check interface is up
ip link show eno1

# Restart network
sudo nmcli connection down lidar-network
sudo nmcli connection up lidar-network

# Verify IP
ip addr show eno1
```

### Issue: No UDP data
1. Check LiDAR work mode is **ENET** (not UART)
2. Check LiDAR is in **Normal** mode (not Standby)
3. Power cycle the LiDAR

### Issue: Wrong subnet
If you accidentally configured 192.168.1.x instead of 192.168.2.x:
1. Connect LiDAR via UART
2. Reconfigure IP settings
3. Restart LiDAR

---

## Physical Setup Checklist

- [x] Jetson `eno1` connected to network switch
- [x] Front LiDAR connected to network switch
- [x] Rear LiDAR connected to network switch
- [x] Both LiDARs powered via 12V
- [x] LiDARs labeled physically (Front/Rear)
- [x] Network configuration persistent (NetworkManager)
- [x] Both LiDARs tested individually
- [x] Both LiDARs can stream simultaneously

---

## Important Notes

⚠️ **Subnet Separation:**
- WiFi network: `192.168.1.0/24` (main network)
- LiDAR network: `192.168.2.0/24` (isolated)
- This prevents routing conflicts

✓ **Port Differentiation:**
- Each LiDAR sends to a different port (6201 vs 6202)
- This allows distinguishing data streams in software

✓ **Network Persistence:**
- Configuration survives reboots
- LiDARs remember settings (stored in flash)
- NetworkManager auto-connects on boot

---

**Status:** Hardware setup complete and tested ✅  
**Next Phase:** ROS 2 driver integration (Phase 1 - Core Localization)

