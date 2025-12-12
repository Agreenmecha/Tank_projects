# Xiaomi Router - Simple Local Access Point

**Purpose:** WiFi access to rover's local network (192.168.2.x) - NO INTERNET  
**Use Case:** SSH into Jetson wirelessly or connect laptop via ethernet to switch

---

## Network Layout (Simple)

```
    Network Switch (192.168.2.0/24)
           |
    +------+-------+--------+
    |      |       |        |
 Jetson  Router  LiDAR  LiDAR   [Your Laptop]
 .100    .1      .62    .63     (WiFi or Ethernet)
```

**All devices on same flat network: 192.168.2.x**

---

## Quick Setup (3 Steps)

### Step 1: Find Router

```bash
cd /home/aaronjet/Tank_projects
./scripts/network/find_router.sh
```

### Step 2: Access & Configure Router

If router is on default network:
```bash
sudo ./scripts/network/access_xiaomi_default.sh
```

In router web interface:
1. **Network → LAN Settings:**
   - IP Address: `192.168.2.1`
   - Subnet Mask: `255.255.255.0`
   - Gateway: Leave blank or `192.168.2.1`

2. **DHCP Settings:**
   - Option A: **Enable DHCP** (easier)
     - DHCP Range: `192.168.2.150` to `192.168.2.200`
     - Reserve: .100 (Jetson), .62 (LiDAR Front), .63 (LiDAR Rear)
   
   - Option B: **Disable DHCP** (manual IPs)
     - You manually set IP on each WiFi device

3. **WiFi Settings:**
   - SSID: `Tank_Rover` (or your choice)
   - Password: [your choice]
   - Security: WPA2/WPA3

4. **WAN/Internet:** Disable or ignore

5. **Save & Reboot**

### Step 3: Connect & Test

**Option A: WiFi Connection**
```bash
# On your laptop, connect to WiFi: Tank_Rover
# You'll get IP 192.168.2.X

# Test
ping 192.168.2.100   # Jetson
ssh aaronjet@192.168.2.100
```

**Option B: Direct Ethernet**
```bash
# Plug laptop ethernet cable into switch
# Set manual IP on laptop:
#   IP: 192.168.2.50
#   Subnet: 255.255.255.0
#   Gateway: (leave blank)

# Test
ping 192.168.2.100
ssh aaronjet@192.168.2.100
```

---

## Static IP on Laptop (If Router DHCP Disabled)

### Linux
```bash
# Temporary (WiFi)
sudo ip addr add 192.168.2.50/24 dev wlan0

# Temporary (Ethernet)
sudo ip addr add 192.168.2.50/24 dev eth0
```

### Windows
1. Network Settings → Change Adapter Options
2. Right-click adapter → Properties
3. IPv4 → Properties
4. Use the following IP:
   - IP: `192.168.2.50`
   - Subnet: `255.255.255.0`
   - Gateway: (leave blank)

### Mac
1. System Preferences → Network
2. Select adapter → Advanced → TCP/IP
3. Configure IPv4: Manually
   - IP: `192.168.2.50`
   - Subnet: `255.255.255.0`
   - Router: (leave blank)

---

## IP Address Plan

| Device | IP | Notes |
|--------|-----|-------|
| Jetson | 192.168.2.100 | Fixed (already configured) |
| Router | 192.168.2.1 | Configure in router settings |
| LiDAR Front | 192.168.2.62 | Fixed (already configured) |
| LiDAR Rear | 192.168.2.63 | Fixed (already configured) |
| Your Laptop | 192.168.2.50 | Manual or DHCP (150-200) |
| Other Devices | 192.168.2.51-60 | Manual assignments |

---

## Simplified DHCP Setup (Optional)

If you want the router to provide DHCP (easiest option):

**Just enable DHCP in router settings:**
- DHCP Range: `192.168.2.150-200`
- Gateway: `192.168.2.1` (router itself)
- DNS: Can leave blank (no internet anyway)

**No need to run the Jetson DHCP script** - the router handles it.

---

## Access Methods

### Method 1: WiFi
```bash
# Connect to: Tank_Rover
# Auto IP via DHCP or manual: 192.168.2.50
ssh aaronjet@192.168.2.100
```

### Method 2: Direct Ethernet
```bash
# Laptop → Switch (ethernet cable)
# Manual IP: 192.168.2.50/24
ssh aaronjet@192.168.2.100
```

### Method 3: Monitor + Keyboard
```bash
# Physical connection to Jetson
# No network needed
```

---

## Troubleshooting

### Can't find router
```bash
./scripts/network/find_router.sh

# If still not found, try:
sudo ./scripts/network/access_xiaomi_default.sh
```

### Can't ping Jetson after connecting
```bash
# Check your IP
ip addr    # Linux/Mac
ipconfig   # Windows

# Should be 192.168.2.X
# If not, set manual IP: 192.168.2.50/24
```

### WiFi connected but can't access devices
```bash
# Check subnet - must be 192.168.2.x
# Verify router DHCP is giving correct range
# Or set manual IP: 192.168.2.50
```

### Router web interface not accessible
```bash
# After initial config, access at:
http://192.168.2.1

# If that fails:
sudo ./scripts/network/access_xiaomi_default.sh
```

---

## ROS2 Over WiFi (After Setup)

```bash
# On laptop (connected to Tank_Rover WiFi)
export ROS_DOMAIN_ID=0

# List topics
ros2 topic list

# View LiDAR data
ros2 topic echo /lidar_front/pointcloud

# Run RViz
rviz2
```

---

## Quick Reference

**Router Web Interface:** http://192.168.2.1  
**Router WiFi:** Tank_Rover (or your SSID)  
**Jetson SSH:** ssh aaronjet@192.168.2.100  
**Network:** 192.168.2.0/24 (all devices)

**Scripts:**
- `./scripts/network/find_router.sh` - Find router
- `sudo ./scripts/network/access_xiaomi_default.sh` - Access on default network
- `sudo ./scripts/network/restore_lidar_network.sh` - Restore if needed

**No Need For:**
- ❌ Internet sharing
- ❌ NAT/routing
- ❌ Complex firewall rules
- ❌ Jetson DHCP server (unless you want it)

**Just Need:**
- ✅ Router on 192.168.2.1
- ✅ Router DHCP enabled (or manual IPs)
- ✅ WiFi enabled with password
- ✅ All devices on 192.168.2.x subnet

---

## Summary

This is a **simple local network** for rover control:
- All devices on same subnet (192.168.2.x)
- No internet connection
- WiFi for wireless access
- Ethernet for direct connection
- Simple, fast, reliable

**Total setup time: 10 minutes**

---

**Last Updated:** December 10, 2025

