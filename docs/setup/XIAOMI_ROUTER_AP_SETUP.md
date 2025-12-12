# Xiaomi BE3600 Router WiFi 7 Access Point Setup

**Date:** December 10, 2025  
**Hardware:** Xiaomi BE3600 Router 2.5G Version (WiFi 7) + Jetson Orin Nano  
**Purpose:** Create wireless access point for rover control while maintaining lidar network

---

## Network Architecture

```
Internet (Optional)
    |
    v
Jetson Orin Nano (wlP1p1s0: 192.168.1.167) ← Main WiFi (NETGEAR01)
    |
    | eno1: 192.168.2.100
    |
    v
Network Switch/Router (192.168.2.0/24)
    |
    +-- Xiaomi BE3600 Router (AP Mode)
    |   └── WiFi: Tank_Rover_AP (192.168.2.x)
    |
    +-- LiDAR Front (192.168.2.62)
    |
    +-- LiDAR Rear (192.168.2.63)
```

---

## Configuration Goals

1. **Xiaomi Router in AP Mode**: Provides WiFi access to the lidar network
2. **No DHCP Conflict**: Router acts as bridge, not DHCP server
3. **Same Subnet**: WiFi clients get 192.168.2.x addresses
4. **Rover Control**: Wireless devices can communicate with Jetson and sensors

---

## Step 1: Find the Router's IP Address

The router should have received an IP from your network. Let's find it:

```bash
# Method 1: Check ARP table
arp -a | grep 192.168.2

# Method 2: Scan the network (install nmap if needed)
sudo apt-get install -y nmap
sudo nmap -sn 192.168.2.0/24

# Method 3: Check DHCP leases (if router is acting as DHCP server)
# Look for a device that's not .62, .63, or .100
```

**Expected Output:**
```
? (192.168.2.1) at XX:XX:XX:XX:XX:XX [ether] on eno1
```

If the router is at a different IP (like 192.168.31.1 - Xiaomi default), you'll need to connect directly.

---

## Step 2: Access Router Web Interface

### Option A: Router Has IP on 192.168.2.x Network

If you found the router's IP in Step 1:

```bash
# Open browser to router's IP
firefox http://192.168.2.1
# or
firefox http://192.168.2.X  # Replace X with discovered IP
```

### Option B: Router Using Default IP (192.168.31.1)

If the router is brand new or reset, it may be on its default network:

1. **Temporarily configure Jetson for Xiaomi's default network:**

```bash
# Backup current config
nmcli connection show lidar-network > ~/lidar-network-backup.txt

# Temporarily change Jetson IP to access router
sudo nmcli connection modify lidar-network ipv4.addresses 192.168.31.100/24
sudo nmcli connection down lidar-network
sudo nmcli connection up lidar-network

# Access router
firefox http://192.168.31.1
```

2. **After configuration, restore original network:**

```bash
sudo nmcli connection modify lidar-network ipv4.addresses 192.168.2.100/24
sudo nmcli connection down lidar-network
sudo nmcli connection up lidar-network
```

---

## Step 3: Initial Router Login

### Default Credentials (Xiaomi BE3600)

- **URL:** `http://192.168.31.1` (default) or `miwifi.com`
- **Default Password:** Usually printed on router label, or:
  - Check bottom of router
  - Try: `admin` / `admin`
  - Or set password on first login

### First Time Setup

If this is the first time accessing the router:

1. **Language:** Select English
2. **Admin Password:** Create a strong password (save it!)
3. **WiFi Name (SSID):** `Tank_Rover_AP` (or your preference)
4. **WiFi Password:** Create a strong WiFi password (save it!)
5. **Internet Connection:** Skip or select "Static IP" (we'll configure later)

---

## Step 4: Configure Router as Access Point

### Method 1: Using Router's AP Mode (Recommended)

Most Xiaomi routers have a built-in AP mode:

1. **Login to router web interface**
2. **Navigate to:** Settings → Network Settings → Work Mode
3. **Select:** "Access Point Mode" or "AP Mode" or "Wireless Bridge Mode"
4. **Configure:**
   - **Router IP:** `192.168.2.1` (static)
   - **Subnet Mask:** `255.255.255.0`
   - **Gateway:** `192.168.2.100` (Jetson IP)
   - **DNS:** `8.8.8.8, 8.8.4.4` (Google DNS) or your preferred DNS
5. **WiFi Settings:**
   - **SSID (2.4GHz):** `Tank_Rover_AP_2G`
   - **SSID (5GHz):** `Tank_Rover_AP_5G`
   - **SSID (6GHz):** `Tank_Rover_AP_6G` (WiFi 7)
   - **Password:** Same for all bands (for simplicity)
   - **Security:** WPA3-Personal (or WPA2/WPA3 mixed)
6. **DHCP Server:** **DISABLE** (important!)
7. **Save and Reboot**

### Method 2: Manual Configuration (If No AP Mode)

If the router doesn't have a dedicated AP mode:

1. **Set Static IP:**
   - Go to: Network Settings → LAN Settings
   - **IP Address:** `192.168.2.1`
   - **Subnet Mask:** `255.255.255.0`
   - **Gateway:** Leave blank or set to `192.168.2.100`

2. **Disable DHCP:**
   - Go to: Network Settings → DHCP Server
   - **DHCP Server:** Disable/Off
   - Save changes

3. **Configure WiFi:**
   - Go to: WiFi Settings
   - **2.4GHz:**
     - SSID: `Tank_Rover_AP_2G`
     - Channel: Auto or 6
     - Bandwidth: 40MHz
     - Security: WPA2/WPA3
     - Password: [your password]
   - **5GHz:**
     - SSID: `Tank_Rover_AP_5G`
     - Channel: Auto or 149
     - Bandwidth: 80MHz
     - Security: WPA2/WPA3
     - Password: [same password]
   - **6GHz (WiFi 7):**
     - SSID: `Tank_Rover_AP_6G`
     - Channel: Auto
     - Bandwidth: 160MHz
     - Security: WPA3
     - Password: [same password]

4. **WAN Settings:**
   - Go to: Network Settings → WAN Settings
   - **Connection Type:** Static IP (or Disable WAN)
   - This prevents the router from trying to get internet

5. **Save and Reboot**

---

## Step 5: Configure Jetson for DHCP Server (Optional)

Since the router won't be providing DHCP, you can configure the Jetson to provide IP addresses to WiFi clients:

### Install and Configure dnsmasq

```bash
# Install dnsmasq (lightweight DHCP/DNS server)
sudo apt-get update
sudo apt-get install -y dnsmasq

# Backup original config
sudo cp /etc/dnsmasq.conf /etc/dnsmasq.conf.backup

# Create new configuration
sudo tee /etc/dnsmasq.conf > /dev/null << 'EOF'
# DHCP Server for Tank Rover Network
interface=eno1
bind-interfaces

# DHCP range for WiFi clients
dhcp-range=192.168.2.150,192.168.2.200,255.255.255.0,24h

# Static IPs for known devices
dhcp-host=0c:29:ab:7c:00:01,192.168.2.62,lidar-front
dhcp-host=0c:29:ab:7c:00:01,192.168.2.63,lidar-rear

# Gateway (Jetson itself for internet sharing)
dhcp-option=3,192.168.2.100

# DNS servers
dhcp-option=6,8.8.8.8,8.8.4.4

# Domain name
domain=tank.local

# Enable logging
log-dhcp
log-queries
EOF

# Enable and start dnsmasq
sudo systemctl enable dnsmasq
sudo systemctl restart dnsmasq

# Check status
sudo systemctl status dnsmasq
```

### Enable IP Forwarding (For Internet Sharing)

If you want WiFi clients to access the internet through the Jetson:

```bash
# Enable IP forwarding
sudo sysctl -w net.ipv4.ip_forward=1

# Make it permanent
echo "net.ipv4.ip_forward=1" | sudo tee -a /etc/sysctl.conf

# Configure NAT (Network Address Translation)
sudo iptables -t nat -A POSTROUTING -o wlP1p1s0 -j MASQUERADE
sudo iptables -A FORWARD -i eno1 -o wlP1p1s0 -j ACCEPT
sudo iptables -A FORWARD -i wlP1p1s0 -o eno1 -m state --state RELATED,ESTABLISHED -j ACCEPT

# Save iptables rules
sudo apt-get install -y iptables-persistent
sudo netfilter-persistent save
```

---

## Step 6: Verify Configuration

### Check Router Status

```bash
# Ping router
ping -c 3 192.168.2.1

# Check if router responds on HTTP
curl -I http://192.168.2.1
```

### Scan for WiFi Networks

```bash
# Scan for your new WiFi network
nmcli device wifi list | grep Tank_Rover

# Should show:
# Tank_Rover_AP_2G  (2.4GHz)
# Tank_Rover_AP_5G  (5GHz)
# Tank_Rover_AP_6G  (6GHz - WiFi 7)
```

### Test WiFi Connection

From a laptop or phone:

1. **Connect to WiFi:**
   - SSID: `Tank_Rover_AP_5G` (recommended for speed)
   - Password: [your WiFi password]

2. **Check IP Address:**
   - Should receive IP in range: `192.168.2.150-200` (if DHCP enabled)
   - Or manually set: `192.168.2.X` (X = 101-149 or 201-254)

3. **Test Connectivity:**
   ```bash
   # Ping Jetson
   ping 192.168.2.100
   
   # Ping LiDARs
   ping 192.168.2.62
   ping 192.168.2.63
   
   # Access Jetson via SSH
   ssh aaronjet@192.168.2.100
   ```

---

## Step 7: Configure Static Routes (If Needed)

If WiFi clients need to access both the lidar network and internet:

### On WiFi Client (Laptop/Phone)

**Linux/Mac:**
```bash
# Add route to lidar network via Jetson
sudo route add -net 192.168.2.0/24 gw 192.168.2.100

# For internet, use default gateway
# (automatically configured via DHCP)
```

**Windows:**
```cmd
# Add route to lidar network
route ADD 192.168.2.0 MASK 255.255.255.0 192.168.2.100

# View routes
route PRINT
```

---

## Troubleshooting

### Issue: Cannot Access Router at 192.168.2.1

**Solution 1: Router still on default network**
```bash
# Temporarily change Jetson IP
sudo nmcli connection modify lidar-network ipv4.addresses 192.168.31.100/24
sudo nmcli connection down lidar-network && sudo nmcli connection up lidar-network

# Access router at http://192.168.31.1
# Configure it, then restore Jetson IP
sudo nmcli connection modify lidar-network ipv4.addresses 192.168.2.100/24
sudo nmcli connection down lidar-network && sudo nmcli connection up lidar-network
```

**Solution 2: Find router's current IP**
```bash
# Scan network
sudo nmap -sn 192.168.2.0/24

# Check ARP table
arp -a | grep eno1
```

### Issue: WiFi Network Not Visible

**Check:**
1. Router is powered on (12V or PoE)
2. WiFi is enabled in router settings
3. Router has completed boot (wait 2-3 minutes)
4. Try scanning from different device

**Force WiFi scan:**
```bash
sudo nmcli device wifi rescan
nmcli device wifi list
```

### Issue: Can Connect to WiFi but No Network Access

**Check DHCP:**
```bash
# On Jetson, check dnsmasq status
sudo systemctl status dnsmasq

# Check dnsmasq logs
sudo journalctl -u dnsmasq -f

# Check DHCP leases
cat /var/lib/misc/dnsmasq.leases
```

**Manual IP Configuration (on WiFi client):**
- IP: `192.168.2.X` (pick unused, e.g., 192.168.2.150)
- Subnet: `255.255.255.0`
- Gateway: `192.168.2.100` (Jetson)
- DNS: `8.8.8.8, 8.8.4.4`

### Issue: Can Ping Jetson but Not LiDARs

**Check routing:**
```bash
# On WiFi client
ip route show
# or
route -n

# Should have route to 192.168.2.0/24
```

**Check firewall on Jetson:**
```bash
sudo ufw status
# If active, allow traffic
sudo ufw allow from 192.168.2.0/24
```

### Issue: DHCP Not Working

**Check dnsmasq:**
```bash
# View logs
sudo journalctl -u dnsmasq -n 50

# Test DHCP manually
sudo dhcpcd -T eno1

# Restart dnsmasq
sudo systemctl restart dnsmasq
```

**Alternative: Use router's DHCP** (if you re-enable it)
- Configure router DHCP range: `192.168.2.150-200`
- Reserve IPs for Jetson (.100), LiDARs (.62, .63)

---

## Advanced Configuration

### WiFi 7 Optimization (6GHz Band)

For maximum performance on the 6GHz band:

1. **Router Settings:**
   - Channel Width: 160MHz or 320MHz
   - MCS: Auto or MCS 13
   - MLO (Multi-Link Operation): Enable
   - BSS Color: Auto

2. **Client Requirements:**
   - WiFi 7 compatible device
   - 6GHz support
   - Updated drivers

### Mesh Networking (Multiple APs)

If you add more Xiaomi routers for extended coverage:

1. **Enable Mesh on Main Router:**
   - Settings → Mesh Network → Enable
   - Set as "Main Router"

2. **Add Satellite Routers:**
   - Connect via Ethernet (preferred) or WiFi
   - Settings → Mesh Network → Add Device
   - Follow pairing instructions

### IoT Device Integration (Mi Home)

For smart home integration:

1. **Install Mi Home App** (on phone)
2. **Add Router to Mi Home:**
   - Open app → Add Device → Network Devices
   - Select "Xiaomi Router"
   - Follow setup wizard
3. **Features:**
   - Remote management
   - Device monitoring
   - Parental controls
   - Guest network

---

## Security Recommendations

### WiFi Security

1. **Use WPA3:** Enable WPA3-Personal for best security
2. **Strong Password:** Minimum 16 characters, mixed case, numbers, symbols
3. **Disable WPS:** Not needed and can be exploited
4. **Hide SSID:** Optional, but adds minor security

### Network Security

1. **Change Admin Password:** Use strong, unique password
2. **Disable Remote Management:** Unless needed
3. **Enable Firewall:** On router (if available in AP mode)
4. **Regular Updates:** Check for firmware updates

### Access Control

```bash
# On Jetson, restrict SSH to specific IPs
sudo ufw allow from 192.168.2.0/24 to any port 22

# Or specific device only
sudo ufw allow from 192.168.2.150 to any port 22
```

---

## Performance Optimization

### WiFi Channel Selection

**2.4GHz Band:**
- Channels: 1, 6, or 11 (non-overlapping)
- Avoid auto if in crowded area
- Use WiFi analyzer to find best channel

**5GHz Band:**
- Channels: 36-48 (indoor), 149-165 (outdoor)
- 80MHz bandwidth recommended
- DFS channels (52-144) may have radar interference

**6GHz Band (WiFi 7):**
- Channels: 1-233 (6GHz spectrum)
- 160MHz or 320MHz bandwidth
- Less interference, better performance

### QoS (Quality of Service)

Configure QoS for rover operations:

1. **High Priority:**
   - ROS2 topics (UDP ports)
   - LiDAR data (ports 6201, 6202)
   - Video streams

2. **Medium Priority:**
   - SSH connections
   - Web interfaces

3. **Low Priority:**
   - File transfers
   - Updates

---

## Quick Reference

### Network Information

| Device | Interface | IP Address | Purpose |
|--------|-----------|------------|---------|
| Jetson | wlP1p1s0 | 192.168.1.167 | Internet (NETGEAR01) |
| Jetson | eno1 | 192.168.2.100 | Lidar Network |
| Router | LAN | 192.168.2.1 | Access Point |
| LiDAR Front | ETH | 192.168.2.62 | Front sensor |
| LiDAR Rear | ETH | 192.168.2.63 | Rear sensor |
| WiFi Clients | WiFi | 192.168.2.150-200 | DHCP range |

### Essential Commands

```bash
# Check network status
ip addr show eno1
nmcli connection show lidar-network

# Scan WiFi networks
nmcli device wifi list | grep Tank

# Test connectivity
ping 192.168.2.1    # Router
ping 192.168.2.62   # Front LiDAR
ping 192.168.2.63   # Rear LiDAR

# DHCP server status
sudo systemctl status dnsmasq
sudo journalctl -u dnsmasq -f

# View DHCP leases
cat /var/lib/misc/dnsmasq.leases

# Restart network
sudo nmcli connection down lidar-network
sudo nmcli connection up lidar-network
```

### Router Access

- **Web Interface:** http://192.168.2.1
- **Default URL:** http://miwifi.com (if on default network)
- **Admin Password:** [your password]
- **WiFi Password:** [your WiFi password]

---

## Testing Checklist

- [ ] Router accessible at 192.168.2.1
- [ ] WiFi networks visible (2.4G, 5G, 6G)
- [ ] Can connect to WiFi from laptop/phone
- [ ] WiFi client receives IP address (DHCP)
- [ ] Can ping Jetson (192.168.2.100) from WiFi
- [ ] Can ping LiDARs (.62, .63) from WiFi
- [ ] Can SSH to Jetson from WiFi
- [ ] Internet access works (if configured)
- [ ] ROS2 topics accessible from WiFi client
- [ ] LiDAR data streams visible

---

## Next Steps

After WiFi is working:

1. **Test ROS2 over WiFi:**
   ```bash
   # On WiFi client
   export ROS_DOMAIN_ID=0
   export ROS_MASTER_URI=http://192.168.2.100:11311
   ros2 topic list
   ros2 topic echo /lidar_front/pointcloud
   ```

2. **Configure RViz on Laptop:**
   - Connect to Tank_Rover_AP WiFi
   - Set ROS_DOMAIN_ID=0
   - Launch RViz
   - Add PointCloud2 displays for LiDARs

3. **Set Up Remote Control:**
   - Install teleop_twist_keyboard
   - Control rover over WiFi
   - Test latency and responsiveness

4. **Monitor Performance:**
   - Check WiFi signal strength
   - Monitor bandwidth usage
   - Test range and coverage

---

## Additional Resources

### Xiaomi Router Documentation
- Official Manual: Check router packaging or Xiaomi website
- Mi WiFi App: Available on iOS/Android for easy management
- Community Forums: Xiaomi Community, Reddit r/Xiaomi

### WiFi 7 Resources
- IEEE 802.11be standard documentation
- WiFi Alliance: WiFi 7 certification info
- Performance benchmarks and comparisons

### ROS2 Networking
- ROS2 DDS configuration for WiFi
- Network performance tuning
- Multi-machine ROS2 setup guides

---

**Document Version:** 1.0  
**Last Updated:** December 10, 2025  
**Author:** Tank Rover Project  
**Status:** Ready for Testing

