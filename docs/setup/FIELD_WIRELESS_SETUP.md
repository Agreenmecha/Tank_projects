# Field Wireless Access Point Setup

**Goal:** Connect laptop wirelessly to Jetson in the field for monitoring, control, and RViz visualization

---

## Network Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Network Switch                           │
│                   (192.168.2.0/24)                          │
└──────┬─────────┬──────────┬──────────┬─────────────────────┘
       │         │          │          │
   Jetson    LiDAR     LiDAR      Wireless AP
   eno1      Front     Rear      (192.168.2.1)
 .2.100     .2.62     .2.63           │
                                      │ WiFi
                                      │
                              ┌───────▼────────┐
                              │  Laptop/Tablet │
                              │  192.168.2.50  │
                              │  (DHCP)        │
                              └────────────────┘
```

---

## Hardware Setup

### Components Needed
- ✅ Network switch (already have)
- ✅ Wireless Access Point (you have)
- ✅ Ethernet cable (AP to switch)
- ✅ Power for AP (PoE or separate power)

### Physical Connections
1. Jetson `eno1` → Switch port 1
2. LiDAR Front → Switch port 2
3. LiDAR Rear → Switch port 3
4. **Wireless AP → Switch port 4**

---

## Access Point Configuration

### Recommended AP Settings

**Network Configuration:**
```
Mode:           Access Point (AP mode, not router)
SSID:           Tank_Field (or your choice)
Password:       <your_secure_password>
Security:       WPA2-PSK or WPA3

IP Address:     192.168.2.1
Subnet Mask:    255.255.255.0
Gateway:        192.168.2.1 (itself)

DHCP Server:    Enabled
DHCP Range:     192.168.2.50 - 192.168.2.100
Lease Time:     1 hour

DNS:            192.168.2.1 (or 8.8.8.8 if internet via AP)
```

**Why these settings?**
- Same subnet as Jetson/LiDARs (192.168.2.x)
- DHCP auto-assigns IPs to connecting devices
- No routing/NAT needed (simple Layer 2 network)

### Popular AP Models Configuration

#### TP-Link EAP Series (Omada)
```
1. Connect AP to switch
2. Access web interface: http://192.168.0.254 (default)
3. Quick Setup:
   - Operation Mode: Access Point
   - Wireless → SSID: Tank_Field
   - Wireless → Security: WPA2-PSK
   - Network → LAN → IP: 192.168.2.1
   - Network → DHCP → Enable: Yes
   - Network → DHCP → Range: 192.168.2.50-192.168.2.100
4. Save & Reboot
```

#### Ubiquiti UniFi
```
1. Adopt AP in UniFi Controller
2. Settings → Networks → Create New:
   - Name: Tank_Field_Network
   - VLAN: None
   - Gateway: 192.168.2.1
   - Subnet: 192.168.2.0/24
   - DHCP: Enabled (192.168.2.50-192.168.2.100)
3. Settings → WiFi → Create New:
   - Name: Tank_Field
   - Security: WPA2-PSK
   - Network: Tank_Field_Network
4. Apply
```

#### MikroTik hAP
```
/interface bridge add name=bridge1
/interface bridge port add bridge=bridge1 interface=ether1
/interface bridge port add bridge=bridge1 interface=wlan1
/ip address add address=192.168.2.1/24 interface=bridge1
/ip pool add name=dhcp_pool ranges=192.168.2.50-192.168.2.100
/ip dhcp-server add address-pool=dhcp_pool interface=bridge1 name=dhcp1
/ip dhcp-server network add address=192.168.2.0/24 gateway=192.168.2.1
/interface wireless set wlan1 ssid="Tank_Field" mode=ap-bridge
/interface wireless security-profiles add name=tank authentication-types=wpa2-psk mode=dynamic-keys wpa2-pre-shared-key="<password>"
/interface wireless set wlan1 security-profile=tank
```

#### Generic/Consumer Router in AP Mode
```
1. Access router web interface (usually 192.168.1.1)
2. Change router mode to "Access Point" or "AP Mode"
   (Disables routing/NAT, becomes simple switch+WiFi)
3. Set LAN IP to: 192.168.2.1
4. Set Subnet: 255.255.255.0
5. Enable DHCP Server:
   - Start IP: 192.168.2.50
   - End IP: 192.168.2.100
6. Configure WiFi:
   - SSID: Tank_Field
   - Security: WPA2-PSK
   - Password: <your_password>
7. Save & Reboot
```

---

## Jetson Configuration

### No Changes Needed! ✅

Jetson already configured:
- IP: `192.168.2.100` on `eno1`
- Subnet: `192.168.2.0/24`
- ROS_DOMAIN_ID: `42`

Current config:
```bash
# Check configuration
ip addr show eno1
# Should show: inet 192.168.2.100/24
```

---

## Laptop/Desktop Field Setup

### Connect to Field Network

**1. Connect to WiFi:**
- SSID: `Tank_Field`
- Password: `<your_password>`

**2. Verify IP assignment:**
```bash
# On laptop
ip addr show wlan0  # or wlp3s0, etc.
# Should get: 192.168.2.50-192.168.2.100 range
```

**3. Test connectivity:**
```bash
# Ping Jetson
ping 192.168.2.100

# Ping LiDARs
ping 192.168.2.62
ping 192.168.2.63
```

### SSH to Jetson

```bash
ssh aaronjet@192.168.2.100
# Password: midang#1
```

### Launch RViz for Field Visualization

**On Laptop:**
```bash
# Set same ROS_DOMAIN_ID
export ROS_DOMAIN_ID=42

# Launch RViz
source /opt/ros/humble/setup.bash
rviz2
```

**In RViz:**
- Add → PointCloud2 → `/lidar_front/pointcloud`
- Add → PointCloud2 → `/lidar_rear/pointcloud`
- Fixed Frame: `lidar_front`

---

## Field Operations Workflow

### 1. Power On Tank

```
1. Power on Jetson
2. Power on LiDARs (12V)
3. Power on Wireless AP
4. Wait 30 seconds for boot
```

### 2. Connect from Laptop

```bash
# Connect to Tank_Field WiFi
# Verify: ping 192.168.2.100

# SSH to Jetson
ssh aaronjet@192.168.2.100

# On Jetson - Start LiDARs
cd ~/Tank_projects/tank_ws
source install/setup.bash
ros2 launch tank_sensors dual_lidar.launch.py
```

### 3. Visualize on Laptop

```bash
# On laptop - separate terminal
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
rviz2

# Or run entire autonomous stack remotely
```

### 4. Monitor Status

```bash
# Check LiDAR topics
ros2 topic list

# Monitor data rates
ros2 topic hz /lidar_front/pointcloud

# Check system status
ssh aaronjet@192.168.2.100
htop
```

---

## Advanced: Dual Network Configuration

If you want to keep both WiFi networks available:

### Keep Home WiFi + Add Field AP

**Jetson has two interfaces:**
- `wlP1p1s0`: Home WiFi (`192.168.1.167`) - For development
- `eno1`: Field network (`192.168.2.100`) - For tank operations

**This is already your setup!** ✓

When in the field:
- LiDARs always on `192.168.2.0/24` (eno1)
- Laptop connects to field AP → `192.168.2.0/24`
- Everything works on same subnet

When at home:
- LiDARs still on `192.168.2.0/24` (eno1)
- Laptop connects to home WiFi → `192.168.1.0/24`
- Need laptop to be on BOTH networks (see below)

### Option: Laptop with USB Ethernet Adapter

For simultaneous home WiFi + field LiDAR access:

```
Laptop WiFi → Home network (192.168.1.x) - Internet
Laptop USB-Eth → Field switch (192.168.2.x) - LiDARs
```

Configure USB Ethernet:
```bash
# On laptop
sudo nmcli connection add \
    type ethernet \
    con-name tank-field \
    ifname eth0 \
    ipv4.method manual \
    ipv4.addresses 192.168.2.50/24
```

---

## Security Considerations

### Field Network Isolation

**Good:** Field network is isolated
- No internet access (unless AP has uplink)
- Can't be accessed from home network
- Simple firewall: only allow 192.168.2.0/24

### Access Point Password

**Recommended:**
- WPA2-PSK minimum
- WPA3 if supported
- Strong password (12+ characters)
- Change default AP admin password

### SSH Keys (Optional)

```bash
# On laptop
ssh-keygen -t ed25519

# Copy to Jetson
ssh-copy-id aaronjet@192.168.2.100

# Now can SSH without password
ssh aaronjet@192.168.2.100
```

---

## Troubleshooting

### Issue: Can't Connect to Tank_Field WiFi

**Check:**
1. AP powered on?
2. AP connected to switch with Ethernet?
3. AP DHCP enabled?
4. Correct WiFi password?

**Test:**
```bash
# From laptop connected to Tank_Field
# Check assigned IP
ip addr show wlan0

# Should see: inet 192.168.2.X/24
```

### Issue: Connected to WiFi but Can't Reach Jetson

**Check:**
1. Jetson powered on?
2. Jetson `eno1` connected to same switch?

**Test:**
```bash
# From laptop
ping 192.168.2.1    # AP gateway - should work
ping 192.168.2.100  # Jetson - should work
ping 192.168.2.62   # Front LiDAR - should work

# If AP works but others don't:
# - Check switch has power
# - Check all Ethernet cables connected
```

### Issue: ROS 2 Topics Not Visible

**Check:**
```bash
# On laptop
echo $ROS_DOMAIN_ID
# Should be: 42

# If not set:
export ROS_DOMAIN_ID=42

# Restart ROS 2 daemon
ros2 daemon stop
ros2 daemon start

# Check again
ros2 topic list
```

### Issue: Slow WiFi / Lag in RViz

**Solutions:**
1. Move closer to AP
2. Use 5GHz WiFi band (if supported)
3. Reduce point cloud rate:
   ```python
   # In launch file
   {'cloud_scan_num': 36}  # Fewer points
   ```
4. Use wired connection (laptop USB-Ethernet adapter)

---

## Field Network IP Allocation

| Device | IP Address | Purpose |
|--------|------------|---------|
| Wireless AP | 192.168.2.1 | Gateway, DHCP Server |
| LiDAR Front | 192.168.2.62 | Front point cloud sensor |
| LiDAR Rear | 192.168.2.63 | Rear point cloud sensor |
| Jetson eno1 | 192.168.2.100 | Main compute |
| DHCP Pool | 192.168.2.50-99 | Laptops, tablets, phones |
| Reserved | 192.168.2.101-199 | Future sensors |
| Reserved | 192.168.2.200-254 | Other devices |

---

## Power Considerations

### Access Point Power Options

**Option 1: PoE (Power over Ethernet)**
- Cleanest solution
- Requires PoE switch or PoE injector
- No extra power cable needed

```
PoE Injector → Switch → AP (powered via Ethernet)
```

**Option 2: Separate Power Supply**
- 12V or 5V (check AP specs)
- Requires power outlet or battery
- More cables but simpler

**Option 3: USB Power**
- If AP supports USB power (5V)
- Can power from USB battery bank
- Portable for field use

### Tank Power Architecture

```
Main Battery (12V)
    ├─→ Jetson (via voltage regulator if needed)
    ├─→ LiDAR Front (12V)
    ├─→ LiDAR Rear (12V)
    └─→ Wireless AP (12V or via PoE injector)
```

---

## Testing Checklist

Before going to field:

- [ ] AP configured on 192.168.2.1
- [ ] AP DHCP range: 192.168.2.50-100
- [ ] AP connected to switch
- [ ] Laptop can connect to Tank_Field WiFi
- [ ] Laptop auto-gets IP in DHCP range
- [ ] Can ping Jetson: `ping 192.168.2.100`
- [ ] Can SSH to Jetson: `ssh aaronjet@192.168.2.100`
- [ ] ROS_DOMAIN_ID=42 set on laptop
- [ ] `ros2 topic list` shows LiDAR topics
- [ ] RViz can display point clouds
- [ ] Test at home before going to field!

---

## Field Usage Tips

1. **Pre-configure laptop** with ROS_DOMAIN_ID in `~/.bashrc`
2. **Save RViz config** for quick field startup
3. **Test everything** at home first
4. **Bring USB-Ethernet adapter** as backup
5. **Have phone hotspot** as emergency access
6. **Label cables** - field troubleshooting is harder
7. **Weatherproof AP** if outdoor use

---

## Comparison: Field AP vs Home WiFi

| Feature | Field AP (192.168.2.x) | Home WiFi (192.168.1.x) |
|---------|------------------------|-------------------------|
| **LiDAR Access** | ✅ Direct | ❌ Different subnet |
| **Portability** | ✅ Goes with tank | ❌ Stationary |
| **Internet** | ❌ None (isolated) | ✅ Available |
| **Range** | ⚠️ Limited to AP | ✅ Whole house |
| **Field Use** | ✅ Perfect | ❌ Not available |
| **Security** | ✅ Isolated | ⚠️ Shared network |

**Recommendation:** Use field AP for autonomous operations, home WiFi for development.

---

## Future Enhancements

### Add Camera View
```bash
# Stream camera to laptop
# TODO: Add e-CAM25 CUONX integration
```

### Add Telemetry Dashboard
```bash
# Web-based dashboard on Jetson
# Access: http://192.168.2.100:8080
# TODO: Implement with Foxglove or PlotJuggler
```

### Add Emergency Stop Button
```bash
# Wireless e-stop via network
ros2 topic pub /emergency_stop std_msgs/Bool "{data: true}"
```

---

**Status:** Documentation ready for field AP setup  
**Next:** Configure your wireless AP and test at home before field deployment

