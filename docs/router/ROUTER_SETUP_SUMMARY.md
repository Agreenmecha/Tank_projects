# Router Setup Summary

**Date:** December 10, 2025  
**Status:** Ready for Configuration  
**Router:** Xiaomi BE3600 Router 2.5G Version (WiFi 7)

---

## What We've Created

### 📄 Documentation

1. **Complete Setup Guide:** `docs/setup/XIAOMI_ROUTER_AP_SETUP.md`
   - Detailed configuration instructions
   - Network architecture diagrams
   - Troubleshooting guide
   - Security recommendations
   - WiFi 7 optimization tips

2. **Quick Start Guide:** `docs/setup/ROUTER_QUICKSTART.md`
   - Simple 5-step setup process
   - Quick reference commands
   - Common troubleshooting

### 🛠️ Helper Scripts

All scripts located in: `scripts/network/`

1. **find_router.sh**
   - Scans network to locate router
   - Tests common router IPs
   - Shows all connected devices

2. **access_xiaomi_default.sh**
   - Temporarily switches to Xiaomi's default network (192.168.31.x)
   - Opens router web interface
   - Guides through configuration
   - Automatically restores original network

3. **setup_router_dhcp.sh**
   - Installs and configures dnsmasq (DHCP server)
   - Sets up DHCP range: 192.168.2.150-200
   - Enables internet sharing (NAT)
   - Configures DNS resolution

4. **restore_lidar_network.sh**
   - Restores original network configuration
   - Verifies connectivity to LiDARs and router

---

## Current Network Status

### Existing Configuration

✅ **Jetson Orin Nano:**
- Interface: `eno1`
- IP: `192.168.2.100/24`
- Status: Active

✅ **LiDAR Front:**
- IP: `192.168.2.62`
- Port: 6201
- Status: Responding

✅ **LiDAR Rear:**
- IP: `192.168.2.63`
- Port: 6202
- Status: Responding

⏳ **Xiaomi Router:**
- Target IP: `192.168.2.1`
- Status: **Needs Configuration**

---

## Next Steps to Get WiFi Working

### Step 1: Find the Router (2 minutes)

```bash
cd /home/aaronjet/Tank_projects
./scripts/network/find_router.sh
```

**Expected Results:**
- If router shows up at 192.168.2.x → Go to Step 2A
- If router not found → Go to Step 2B

### Step 2A: Router Already on Network (5 minutes)

If router is already on 192.168.2.x network:

```bash
# Access router directly
firefox http://192.168.2.X  # Use IP from Step 1
```

Then configure as shown in Step 3.

### Step 2B: Router on Default Network (10 minutes)

If router is brand new or on default network:

```bash
# This script handles everything
sudo ./scripts/network/access_xiaomi_default.sh
```

The script will:
1. Temporarily change your network
2. Open router web interface
3. Wait for you to configure
4. Restore original network

### Step 3: Configure Router (5 minutes)

In router web interface:

1. **Login** (password on router label)

2. **Set Static IP:**
   - Settings → Network → LAN
   - IP: `192.168.2.1`
   - Subnet: `255.255.255.0`
   - Gateway: `192.168.2.100`

3. **Disable DHCP:**
   - Settings → DHCP Server → OFF

4. **Setup WiFi:**
   - 2.4GHz: `Tank_Rover_AP_2G`
   - 5GHz: `Tank_Rover_AP_5G`
   - 6GHz: `Tank_Rover_AP_6G`
   - Password: [your choice]
   - Security: WPA2/WPA3

5. **Save & Reboot**

### Step 4: Setup DHCP on Jetson (3 minutes)

```bash
sudo ./scripts/network/setup_router_dhcp.sh
```

This installs and configures dnsmasq to provide:
- DHCP for WiFi clients
- DNS resolution
- Internet sharing

### Step 5: Test WiFi (2 minutes)

**From laptop/phone:**
1. Connect to `Tank_Rover_AP_5G`
2. Enter WiFi password
3. Should get IP: 192.168.2.150-200

**Test connectivity:**
```bash
ping 192.168.2.100  # Jetson
ping 192.168.2.62   # LiDAR Front
ping 192.168.2.63   # LiDAR Rear
ssh aaronjet@192.168.2.100  # SSH to Jetson
```

---

## Network Architecture (After Setup)

```
                    Internet
                       |
                       v
    Jetson WiFi (wlP1p1s0: 192.168.1.167) ← NETGEAR01
                       |
                       | (NAT/Routing)
                       |
    Jetson Ethernet (eno1: 192.168.2.100)
                       |
                       v
              Network Switch/Hub
                       |
        +--------------+---------------+
        |              |               |
   Xiaomi Router   LiDAR Front    LiDAR Rear
   (192.168.2.1)   (192.168.2.62) (192.168.2.63)
        |
        v
   WiFi Clients
   (192.168.2.150-200)
   - Laptop
   - Phone
   - Tablet
```

### Network Features

✅ **Isolated Sensor Network:** LiDARs on dedicated subnet  
✅ **WiFi Access:** Wireless control and monitoring  
✅ **Internet Sharing:** WiFi clients can access internet through Jetson  
✅ **DNS Resolution:** Devices accessible by name (jetson.tank.local)  
✅ **DHCP Management:** Automatic IP assignment for WiFi clients  
✅ **WiFi 7 Support:** 6GHz band for ultra-fast connections

---

## Quick Reference

### IP Address Assignments

| Device | IP Address | Interface | Purpose |
|--------|------------|-----------|---------|
| Jetson | 192.168.2.100 | eno1 | Main controller |
| Router | 192.168.2.1 | LAN | WiFi access point |
| LiDAR Front | 192.168.2.62 | Ethernet | Front sensor |
| LiDAR Rear | 192.168.2.63 | Ethernet | Rear sensor |
| WiFi Clients | 192.168.2.150-200 | WiFi | DHCP range |
| Reserved | 192.168.2.101-149 | - | Static assignments |

### WiFi Networks

| SSID | Band | Speed | Range | Use Case |
|------|------|-------|-------|----------|
| Tank_Rover_AP_2G | 2.4GHz | ~300 Mbps | Long | Remote control |
| Tank_Rover_AP_5G | 5GHz | ~1200 Mbps | Medium | Recommended |
| Tank_Rover_AP_6G | 6GHz | ~2400 Mbps | Short | High bandwidth |

### Useful Commands

```bash
# Network status
ip addr show eno1
nmcli connection show lidar-network

# Find router
./scripts/network/find_router.sh

# Access router (default network)
sudo ./scripts/network/access_xiaomi_default.sh

# Setup DHCP
sudo ./scripts/network/setup_router_dhcp.sh

# Restore network
sudo ./scripts/network/restore_lidar_network.sh

# DHCP status
sudo systemctl status dnsmasq
cat /var/lib/misc/dnsmasq.leases

# View DHCP logs
sudo journalctl -u dnsmasq -f

# Restart services
sudo systemctl restart dnsmasq
sudo nmcli connection down lidar-network
sudo nmcli connection up lidar-network
```

---

## Troubleshooting Quick Fixes

### Router Not Found

```bash
# Check physical connections
# - Router powered on?
# - Ethernet cable connected?
# - Router lights on?

# Rescan network
./scripts/network/find_router.sh
```

### Can't Access Router

```bash
# Use default network access script
sudo ./scripts/network/access_xiaomi_default.sh
```

### WiFi Not Visible

```bash
# Wait 2-3 minutes after router reboot
# Scan from Jetson
nmcli device wifi list | grep Tank

# Check router power and settings
```

### WiFi Connected but No Internet

```bash
# Check DHCP server
sudo systemctl status dnsmasq

# Manually set IP on client:
# IP: 192.168.2.150
# Subnet: 255.255.255.0
# Gateway: 192.168.2.100
# DNS: 8.8.8.8
```

### LiDARs Not Responding

```bash
# Restore network
sudo ./scripts/network/restore_lidar_network.sh

# Test connectivity
ping 192.168.2.62
ping 192.168.2.63
```

---

## Testing Checklist

After setup, verify:

- [ ] Router accessible at http://192.168.2.1
- [ ] WiFi networks visible (2G, 5G, 6G)
- [ ] Can connect to WiFi from laptop
- [ ] WiFi client receives IP (150-200 range)
- [ ] Can ping Jetson from WiFi (192.168.2.100)
- [ ] Can ping LiDARs from WiFi (.62, .63)
- [ ] Can SSH to Jetson from WiFi
- [ ] Internet works on WiFi client
- [ ] DNS resolution works (ping jetson.tank.local)
- [ ] ROS2 topics visible from WiFi client

---

## Performance Tips

### WiFi Band Selection

**Use 2.4GHz when:**
- Operating at long range (>50m)
- Many obstacles between rover and operator
- Low bandwidth requirements (telemetry, commands)

**Use 5GHz when:**
- Medium range (<30m)
- Need good balance of speed and range
- Video streaming, RViz visualization
- **Recommended for most operations**

**Use 6GHz when:**
- Short range (<15m)
- Need maximum bandwidth
- Multiple video streams
- High-frequency sensor data
- Requires WiFi 7 compatible device

### Optimization

1. **Channel Selection:**
   - 2.4GHz: Use channel 1, 6, or 11
   - 5GHz: Use channel 149-165 (outdoor)
   - 6GHz: Auto (less interference)

2. **QoS Priority:**
   - High: ROS2 topics, LiDAR data, video
   - Medium: SSH, web interfaces
   - Low: File transfers, updates

3. **Security:**
   - Use WPA3 (or WPA2/WPA3 mixed)
   - Strong password (16+ characters)
   - Disable WPS
   - Regular firmware updates

---

## What's Next

After WiFi is working:

### 1. Test ROS2 over WiFi

```bash
# On laptop (connected to Tank_Rover_AP)
export ROS_DOMAIN_ID=0
ros2 topic list
ros2 topic echo /lidar_front/pointcloud
```

### 2. Setup RViz Visualization

```bash
# On laptop
ros2 launch tank_sensors dual_lidar.launch.py
rviz2
```

### 3. Remote Control

```bash
# Install teleop on laptop
sudo apt install ros-humble-teleop-twist-keyboard

# Control rover
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 4. Monitor Performance

- Check WiFi signal strength
- Monitor bandwidth usage
- Test latency (ping times)
- Verify range and coverage

---

## Documentation Files

| File | Purpose |
|------|---------|
| `docs/setup/XIAOMI_ROUTER_AP_SETUP.md` | Complete detailed guide |
| `docs/setup/ROUTER_QUICKSTART.md` | Quick 5-step setup |
| `docs/setup/LIDAR_NETWORK_SETUP.md` | LiDAR network reference |
| `ROUTER_SETUP_SUMMARY.md` | This file |

---

## Support

If you encounter issues:

1. **Check the guides:**
   - Quick Start: `docs/setup/ROUTER_QUICKSTART.md`
   - Full Guide: `docs/setup/XIAOMI_ROUTER_AP_SETUP.md`

2. **Run diagnostics:**
   ```bash
   ./scripts/network/find_router.sh
   sudo systemctl status dnsmasq
   ip addr show eno1
   ```

3. **Check logs:**
   ```bash
   sudo journalctl -u dnsmasq -n 50
   sudo journalctl -u NetworkManager -n 50
   ```

4. **Restore to known good state:**
   ```bash
   sudo ./scripts/network/restore_lidar_network.sh
   ```

---

## Summary

✅ **Created:**
- 2 comprehensive documentation files
- 4 helper scripts for easy setup
- Network architecture diagrams
- Troubleshooting guides

⏳ **Ready for:**
- Router configuration (10-15 minutes)
- DHCP setup (3 minutes)
- WiFi testing (2 minutes)

🎯 **Result:**
- Wireless access to rover network
- Remote control and monitoring
- Internet sharing for WiFi clients
- Professional WiFi 7 setup

---

**Total Setup Time:** ~20-30 minutes  
**Difficulty:** Intermediate  
**Status:** Ready to Begin

---

**Last Updated:** December 10, 2025  
**Version:** 1.0

