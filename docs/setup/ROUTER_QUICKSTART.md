# Xiaomi Router Quick Start Guide

**Quick setup guide for getting your Xiaomi BE3600 router working as a WiFi access point**

---

## Current Network Status

Your rover network is currently:
- **Jetson IP:** 192.168.2.100 (eno1 interface)
- **LiDAR Front:** 192.168.2.62
- **LiDAR Rear:** 192.168.2.63
- **Router:** Not configured yet (target: 192.168.2.1)

---

## Quick Setup (3 Steps)

### Step 1: Find the Router

```bash
cd /home/aaronjet/Tank_projects
./scripts/network/find_router.sh
```

This will scan your network and tell you where the router is.

### Step 2: Configure the Router

**Option A: Router on default network (192.168.31.1)**

If the router is brand new or reset:

```bash
sudo ./scripts/network/access_xiaomi_default.sh
```

This script will:
1. Temporarily change your network to access the router
2. Open the router's web interface
3. Guide you through configuration
4. Restore your network when done

**Option B: Router already on 192.168.2.x network**

If the router is already configured on your network:

```bash
# Open browser to router IP
firefox http://192.168.2.X  # Replace X with router's IP
```

### Step 3: Router Configuration

In the router's web interface:

1. **Login** (check router label for password)

2. **Set Router IP:**
   - Go to: Settings → Network Settings → LAN
   - IP Address: `192.168.2.1`
   - Subnet Mask: `255.255.255.0`
   - Gateway: `192.168.2.100` (or leave blank)

3. **Disable DHCP:**
   - Go to: Settings → DHCP Server
   - Turn OFF DHCP Server
   - (Jetson will provide DHCP instead)

4. **Configure WiFi:**
   - Go to: WiFi Settings
   - 2.4GHz SSID: `Tank_Rover_AP_2G`
   - 5GHz SSID: `Tank_Rover_AP_5G`
   - 6GHz SSID: `Tank_Rover_AP_6G` (WiFi 7)
   - Password: [your choice - write it down!]
   - Security: WPA2/WPA3

5. **Save and Reboot**

---

## Step 4: Setup DHCP on Jetson

After router is configured, setup DHCP server on Jetson:

```bash
cd /home/aaronjet/Tank_projects
sudo ./scripts/network/setup_router_dhcp.sh
```

This will:
- Install dnsmasq (DHCP/DNS server)
- Configure DHCP range: 192.168.2.150-200
- Enable internet sharing
- Start the service

---

## Step 5: Test WiFi

### Connect to WiFi

From your laptop or phone:
1. Scan for WiFi networks
2. Connect to: `Tank_Rover_AP_5G` (or 2G/6G)
3. Enter the WiFi password you set
4. You should get IP: 192.168.2.150-200

### Test Connectivity

From your WiFi-connected device:

```bash
# Ping Jetson
ping 192.168.2.100

# Ping LiDARs
ping 192.168.2.62
ping 192.168.2.63

# SSH to Jetson
ssh aaronjet@192.168.2.100
```

---

## Troubleshooting

### Router Not Found

```bash
# Check physical connections
# - Router powered on?
# - Ethernet cable connected to eno1?
# - Router lights blinking?

# Try finding it again
./scripts/network/find_router.sh
```

### Can't Access Router Web Interface

```bash
# If router is on default network (192.168.31.1)
sudo ./scripts/network/access_xiaomi_default.sh

# This will temporarily change your network
```

### WiFi Not Visible

1. Wait 2-3 minutes after router reboot
2. Check router power
3. Verify WiFi is enabled in router settings
4. Try scanning from different device

```bash
# On Jetson, scan for WiFi
nmcli device wifi list | grep Tank
```

### Can Connect to WiFi but No Network

**Check DHCP:**
```bash
# On Jetson
sudo systemctl status dnsmasq
sudo journalctl -u dnsmasq -f
```

**Manual IP (on WiFi client):**
- IP: 192.168.2.150 (or any unused 101-254)
- Subnet: 255.255.255.0
- Gateway: 192.168.2.100
- DNS: 8.8.8.8, 8.8.4.4

### Need to Restore Network

If something goes wrong:

```bash
sudo ./scripts/network/restore_lidar_network.sh
```

---

## Useful Commands

```bash
# Check network status
ip addr show eno1
nmcli connection show lidar-network

# Find router
./scripts/network/find_router.sh

# Check DHCP server
sudo systemctl status dnsmasq
cat /var/lib/misc/dnsmasq.leases

# View DHCP logs
sudo journalctl -u dnsmasq -f

# Restart DHCP
sudo systemctl restart dnsmasq

# Restart network
sudo nmcli connection down lidar-network
sudo nmcli connection up lidar-network
```

---

## Network Scripts

All scripts are in: `/home/aaronjet/Tank_projects/scripts/network/`

| Script | Purpose |
|--------|---------|
| `find_router.sh` | Scan network to find router |
| `access_xiaomi_default.sh` | Access router on default network |
| `restore_lidar_network.sh` | Restore original network config |
| `setup_router_dhcp.sh` | Setup DHCP server on Jetson |

---

## Next Steps

After WiFi is working:

1. **Test ROS2 over WiFi:**
   ```bash
   # On laptop (connected to Tank_Rover_AP WiFi)
   export ROS_DOMAIN_ID=0
   ros2 topic list
   ```

2. **Setup Remote Desktop** (optional):
   ```bash
   # For GUI access to Jetson over WiFi
   sudo apt-get install xrdp
   ```

3. **Configure Firewall** (optional):
   ```bash
   # Allow specific ports only
   sudo ufw enable
   sudo ufw allow from 192.168.2.0/24
   ```

---

## Full Documentation

For detailed information, see:
- **Complete Guide:** `docs/setup/XIAOMI_ROUTER_AP_SETUP.md`
- **LiDAR Network:** `docs/setup/LIDAR_NETWORK_SETUP.md`

---

## Quick Reference

**Network Layout:**
```
192.168.2.100 - Jetson (eno1)
192.168.2.1   - Router (target)
192.168.2.62  - LiDAR Front
192.168.2.63  - LiDAR Rear
192.168.2.150-200 - DHCP range for WiFi clients
```

**WiFi Networks:**
- `Tank_Rover_AP_2G` - 2.4GHz (long range)
- `Tank_Rover_AP_5G` - 5GHz (fast, recommended)
- `Tank_Rover_AP_6G` - 6GHz WiFi 7 (fastest)

**Router Access:**
- Default: http://192.168.31.1 or http://miwifi.com
- After config: http://192.168.2.1

---

**Last Updated:** December 10, 2025

