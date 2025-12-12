# Router Setup - COMPLETE ✅

**Date:** December 10, 2025  
**Router:** Xiaomi BE3600 WiFi 7  
**Status:** Working and Verified

---

## ✅ Verified Working

- ✅ Router accessible at: http://192.168.2.1
- ✅ Router responds to ping: 192.168.2.1
- ✅ Laptop can access router web interface
- ✅ Laptop can SSH to Jetson: ssh aaronjet@192.168.2.100
- ✅ Network connectivity fully operational

---

## Network Configuration

### IP Addresses

| Device | IP Address | Interface |
|--------|------------|-----------|
| Jetson Orin Nano | 192.168.2.100 | eno1 (ethernet) |
| Xiaomi Router | 192.168.2.1 | LAN |
| LiDAR Front | 192.168.2.62 | Ethernet |
| LiDAR Rear | 192.168.2.63 | Ethernet |
| Laptop/Clients | 192.168.2.150-200 | WiFi/Ethernet (DHCP) |

### WiFi Settings

- **SSID:** Aaron_Rover
- **Password:** [configured]
- **Security:** WPA2/WPA3
- **Dual-frequency:** Enabled (2.4GHz, 5GHz, 6GHz use same SSID)
- **Router automatically selects best band**

### Router Settings

- **LAN IP:** 192.168.2.1
- **Subnet Mask:** 255.255.255.0
- **DHCP:** Enabled
- **DHCP Range:** 192.168.2.150 - 192.168.2.200
- **DHCP Lease:** 720 minutes (12 hours)
- **Mode:** Router Mode with local network
- **Internet:** Not configured (local network only)

---

## Network Topology

```
    Network Switch/Hub
    (192.168.2.0/24)
           |
    +------+-------+--------+
    |      |       |        |
 Jetson Router  LiDAR  LiDAR
 .100   .1      .62    .63
         |
      WiFi: Aaron_Rover
         |
    Your Laptop
    (WiFi or Ethernet)
```

---

## Access Methods

### Method 1: WiFi Connection

1. **Connect to WiFi:**
   - SSID: `Aaron_Rover`
   - Password: [your configured password]
   - Device will get IP: 192.168.2.150-200 (automatic)

2. **Access Jetson:**
   ```bash
   ssh aaronjet@192.168.2.100
   ```

3. **Access Router:**
   ```bash
   http://192.168.2.1
   ```

### Method 2: Direct Ethernet

1. **Plug laptop into network switch**
2. **Set manual IP (if needed):**
   - IP: 192.168.2.50
   - Subnet: 255.255.255.0
   - Gateway: (leave blank)

3. **Access Jetson:**
   ```bash
   ssh aaronjet@192.168.2.100
   ```

---

## ROS2 Usage Over Network

### From Laptop (Connected to Network)

```bash
# Set ROS domain
export ROS_DOMAIN_ID=0

# List topics from Jetson
ros2 topic list

# View LiDAR data
ros2 topic echo /lidar_front/pointcloud

# Launch RViz for visualization
rviz2
```

### Configure ROS2 for Network Use

```bash
# On both Jetson and laptop, add to ~/.bashrc
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

# Reload
source ~/.bashrc
```

---

## Useful Commands

### On Jetson

```bash
# Check network status
ip addr show eno1

# Verify connectivity
ping 192.168.2.1     # Router
ping 192.168.2.62    # LiDAR Front
ping 192.168.2.63    # LiDAR Rear

# View DHCP leases (if running dnsmasq)
cat /var/lib/misc/dnsmasq.leases

# Check who's connected
arp -a | grep 192.168.2
```

### On Laptop

```bash
# Check IP address
ip addr          # Linux
ipconfig         # Windows
ifconfig         # Mac

# Test connectivity
ping 192.168.2.100   # Jetson
ping 192.168.2.62    # LiDAR Front
ping 192.168.2.63    # LiDAR Rear

# SSH to Jetson
ssh aaronjet@192.168.2.100
```

### Access Router

```bash
# Web interface
http://192.168.2.1

# Default login: Check router label or use configured password
```

---

## Verified Tests

✅ Router accessible at 192.168.2.1 (ping successful)  
✅ Router web interface accessible from laptop  
✅ SSH to Jetson working from laptop  
✅ All devices on same network (192.168.2.x)  
✅ DHCP providing IP addresses  

---

## Features

### What Works

- ✅ **Wireless Access** - WiFi to rover network
- ✅ **Dual-Band** - All bands (2.4/5/6 GHz) use same SSID
- ✅ **DHCP** - Automatic IP assignment
- ✅ **Local Network** - All devices on same subnet
- ✅ **SSH Access** - Remote terminal access
- ✅ **ROS2 Ready** - Can run ROS2 over network
- ✅ **Direct Ethernet** - Can also connect via cable

### What's NOT Configured

- ❌ Internet access (not needed/configured)
- ❌ DNS resolution (not needed for local network)
- ❌ Port forwarding (not needed)
- ❌ VPN (not needed)

---

## Troubleshooting

### Can't See WiFi Network

```bash
# From device with WiFi
nmcli device wifi list | grep Aaron  # Linux
# or scan from phone/laptop WiFi settings

# If not visible, check router:
# - WiFi enabled?
# - Router powered on?
# - Wait 2-3 minutes after router reboot
```

### Can't Get IP via WiFi

**Check DHCP:**
- Router DHCP enabled?
- Correct range (192.168.2.150-200)?

**Manual IP alternative:**
- IP: 192.168.2.50
- Subnet: 255.255.255.0
- Gateway: (leave blank or 192.168.2.1)

### Can't SSH to Jetson

```bash
# Check Jetson is on network
ping 192.168.2.100

# Verify your laptop IP is 192.168.2.x
ip addr  # Linux
ipconfig # Windows

# Check SSH service on Jetson
systemctl status ssh
```

### Forgot Router Password

- Physical access: Reset button on router
- Or use current admin access to change

---

## Network Security

### Current Setup

- **WiFi Password:** Required (WPA2/WPA3)
- **Router Admin:** Password protected
- **SSH:** Key-based or password authentication
- **Isolated Network:** No internet connection = reduced attack surface

### Recommendations

✅ **Keep WiFi password strong**  
✅ **Keep router admin password secure**  
✅ **Use SSH keys instead of passwords** (optional)  
✅ **Regular router firmware updates** (when available)  
⚠️ **Monitor connected devices** via router interface  

---

## Next Steps

Now that your network is working:

### 1. Test ROS2 Communication

```bash
# On Jetson - start a simple publisher
ros2 topic pub /test std_msgs/String "data: Hello from Jetson"

# On laptop - listen
ros2 topic echo /test
```

### 2. Launch LiDAR Visualization

```bash
# On Jetson - start LiDAR nodes
ros2 launch tank_sensors dual_lidar.launch.py

# On laptop - visualize in RViz
rviz2
# Add PointCloud2 displays for /lidar_front/pointcloud and /lidar_rear/pointcloud
```

### 3. Setup Remote Desktop (Optional)

```bash
# On Jetson - install xrdp for GUI access
sudo apt-get install xrdp
sudo systemctl enable xrdp

# From laptop - use RDP client
# Connect to: 192.168.2.100:3389
```

### 4. Configure Firewall (Optional)

```bash
# On Jetson - allow specific access
sudo ufw enable
sudo ufw allow from 192.168.2.0/24
sudo ufw status
```

---

## Summary

**Your rover network is complete and working!**

- ✅ Xiaomi router provides WiFi access point
- ✅ All devices on 192.168.2.x network
- ✅ Laptop can connect (WiFi or Ethernet)
- ✅ SSH access to Jetson verified
- ✅ Ready for ROS2 remote operation
- ✅ Ready for field use

**Simple, fast, and reliable local network for rover control!**

---

## Quick Reference Card

```
Network:        192.168.2.0/24
Jetson IP:      192.168.2.100
Router IP:      192.168.2.1
WiFi SSID:      Aaron_Rover
DHCP Range:     192.168.2.150-200

SSH Jetson:     ssh aaronjet@192.168.2.100
Router Web:     http://192.168.2.1
LiDAR Front:    192.168.2.62
LiDAR Rear:     192.168.2.63
```

---

**Setup Complete: December 10, 2025**  
**Status: Production Ready ✅**


