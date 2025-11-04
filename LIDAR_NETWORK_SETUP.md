# Dual Unitree L2 LiDAR Network Setup

**Date:** 2025-11-04  
**Hardware:** 2x Unitree L2 LiDARs + Network Switch + Jetson Orin Nano

---

## Network Topology

```
                    Network Switch
                          |
        +-----------------+------------------+
        |                 |                  |
   Jetson Orin      LiDAR Front        LiDAR Rear
  192.168.1.100    192.168.1.62      192.168.1.63
   eno1 interface
```

## IP Configuration Table

| Device | IP Address | UDP Send Port | Target IP | Target Port | Function |
|--------|------------|---------------|-----------|-------------|----------|
| **Jetson** | 192.168.1.100 | - | - | 6201, 6202 | Receives point clouds |
| **LiDAR Front** | 192.168.1.62 | 6101 | 192.168.1.100 | 6201 | Front sensor |
| **LiDAR Rear** | 192.168.1.63 | 6101 | 192.168.1.100 | 6202 | Rear sensor |
| **Gateway** | 192.168.1.1 | - | - | - | Network gateway |
| **Subnet Mask** | 255.255.255.0 | - | - | - | All devices |

---

## Setup Procedure

### Step 1: Configure Jetson Network Interface

Run the setup script:

```bash
cd ~/Tank_projects
chmod +x setup_lidar_network.sh
sudo ./setup_lidar_network.sh
```

**Manual Alternative:**

```bash
# Create NetworkManager connection for eno1
sudo nmcli connection add \
    type ethernet \
    con-name lidar-network \
    ifname eno1 \
    ipv4.method manual \
    ipv4.addresses 192.168.1.100/24 \
    ipv4.gateway 192.168.1.1 \
    autoconnect yes

# Activate
sudo nmcli connection up lidar-network

# Verify
ip addr show eno1
```

### Step 2: Configure First LiDAR (Front) - DEFAULT

The front LiDAR can keep factory defaults:
- ✓ IP: `192.168.1.62` (factory default)
- ✓ Target: `192.168.1.100:6201`

**If you need to change target IP:**

Connect via UART (serial port) using Unilidar 2 software:
1. Connect UART converter to PC
2. Open Unilidar 2 software
3. Select COM port → "Open Serial Port"
4. In **Config Setting**:
   - `Usr ChangeIP`: `192.168.1.100`
   - `Usr ChangePort`: `6201`
5. Click **"Change"** → **"Restart"**
6. Disconnect serial, connect Ethernet

### Step 3: Configure Second LiDAR (Rear) - CHANGE IP

⚠️ **Important:** Configure ONE LiDAR at a time via UART before Ethernet use.

**Via Windows PC + Unilidar 2 Software:**

1. Connect **only LiDAR Rear** via UART converter to Windows PC
2. Power on LiDAR
3. Open Unilidar 2 software
4. Click "Open Serial Port" (select correct COM port)
5. Click **"Synchronous"** to read current config
6. In **Config Setting** module, set:
   - `Lidar ChangeIP`: `192.168.1.63`
   - `Lidar Gateway`: `192.168.1.1`
   - `Lidar SubnetMask`: `255.255.255.0`
   - `Usr ChangeIP`: `192.168.1.100` (Jetson IP)
   - `Usr ChangePort`: `6202` (different from front!)
7. Click **"Change"**
8. Click **"Restart"**
9. Disconnect serial
10. Label this LiDAR as "REAR"

**Via ROS 2 SDK (Alternative - After Initial Setup):**

```bash
# TODO: Use unilidar_sdk2 to configure via network
# This can be done later after SDK integration
```

### Step 4: Physical Connection

1. Power off both LiDARs
2. Connect both LiDARs to network switch via Ethernet
3. Connect network switch to Jetson `eno1` port
4. Power on both LiDARs (12V)
5. Wait 10 seconds for boot

### Step 5: Verify Network Connectivity

```bash
# Ping test
ping -c 3 192.168.1.62  # Front LiDAR
ping -c 3 192.168.1.63  # Rear LiDAR

# Check if LiDARs are sending UDP data
sudo tcpdump -i eno1 udp port 6201 -c 5  # Front
sudo tcpdump -i eno1 udp port 6202 -c 5  # Rear
```

Expected output: You should see UDP packets arriving.

### Step 6: Test Point Cloud Reception

Create a simple UDP listener:

```bash
# Test front LiDAR data
nc -ul 6201 | xxd | head -20

# Test rear LiDAR data  
nc -ul 6202 | xxd | head -20
```

You should see binary point cloud data streaming.

---

## Troubleshooting

### Issue: Cannot ping LiDAR

**Check:**
```bash
# Verify Jetson network config
ip addr show eno1
# Should show: inet 192.168.1.100/24

# Check if interface is up
ip link show eno1
# Should show: state UP

# Restart network
sudo nmcli connection down lidar-network
sudo nmcli connection up lidar-network
```

### Issue: LiDAR has wrong IP

**Solution:** Connect via UART and reconfigure:
1. Use UART converter on Windows PC
2. Open Unilidar 2 → Open Serial Port
3. Click **"Restore Factory Defaults"** if needed
4. Reconfigure IP settings
5. Restart LiDAR

### Issue: No UDP data received

**Check:**
```bash
# Verify firewall allows UDP
sudo ufw status
# If active, allow ports:
sudo ufw allow 6201/udp
sudo ufw allow 6202/udp

# Check if data is arriving
sudo tcpdump -i eno1 -n udp
# Should see packets from 192.168.1.62 and 192.168.1.63
```

**Verify LiDAR configuration** via UART:
- Target IP must be `192.168.1.100` (Jetson)
- Target ports must be `6201` and `6202`
- Work mode must be set to **"ENET"** (not UART)

### Issue: Both LiDARs have same IP (192.168.1.62)

**Problem:** IP conflict, only one LiDAR will work.

**Solution:**
1. Power off both LiDARs
2. Connect only ONE LiDAR via UART to PC
3. Change its IP to `192.168.1.63` following Step 3
4. Label it as "REAR"
5. Now connect both via Ethernet

---

## ROS 2 Integration

After network setup, integrate with ROS 2:

```bash
cd ~/Tank_projects/tank_ws/src/external
git clone https://github.com/unitreerobotics/unilidar_sdk2.git

cd ~/Tank_projects/tank_ws
colcon build --packages-select unilidar_sdk2
source install/setup.bash
```

**Launch file configuration** (to create later in `tank_sensors` package):

```yaml
# Front LiDAR
- lidar_ip: "192.168.1.62"
  lidar_port: 6101
  local_port: 6201
  frame_id: "lidar_front"
  topic: "/lidar_front/pointcloud"

# Rear LiDAR  
- lidar_ip: "192.168.1.63"
  lidar_port: 6101
  local_port: 6202
  frame_id: "lidar_rear"
  topic: "/lidar_rear/pointcloud"
```

---

## Quick Reference Commands

```bash
# Show network config
ip addr show eno1

# Show all LiDAR network traffic
sudo tcpdump -i eno1 net 192.168.1.0/24 -n

# Monitor point cloud data rate
sudo iftop -i eno1 -f "net 192.168.1.0/24"

# Restart network connection
sudo nmcli connection down lidar-network && sudo nmcli connection up lidar-network

# Check active UDP listeners
sudo ss -ulnp | grep -E "6201|6202"
```

---

## Factory Default Settings (Reference)

From Unitree L2 Manual:

```
IP:              192.168.1.62
Gateway:         192.168.1.1
Subnet Mask:     255.255.255.0
Target IP:       192.168.1.2
Radar TX Port:   6101
Target RX Port:  6201
Work Mode:       ENET UDP (default)
```

To restore factory defaults: Connect via UART → Click **"Restore Factory Defaults"** → Restart

---

## Notes

- ⚠️ **Both LiDARs must have UNIQUE IP addresses** on the same subnet
- ⚠️ **Use different target ports** (6201 vs 6202) to distinguish data streams
- ⚠️ The LiDARs send data from their own port **6101** to Jetson's ports **6201/6202**
- ✓ Network switch allows all devices to communicate on same subnet
- ✓ Configuration changes require **serial connection** first time only
- ✓ After initial setup, LiDARs remember settings (stored in flash)

---

**Status:** Ready for hardware setup  
**Next:** Configure rear LiDAR IP via UART, then test network connectivity

