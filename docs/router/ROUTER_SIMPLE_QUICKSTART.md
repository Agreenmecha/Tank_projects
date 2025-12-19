# Router Quick Setup - Local Network Only

**Goal:** WiFi access to rover network - NO INTERNET needed

---

## Your Network (Simple)

```
    Switch/Hub on 192.168.2.0/24
          |
    +-----+------+-------+
    |     |      |       |
 Jetson Router LiDAR LiDAR
 .100   .1     .62   .63
          |
        WiFi → Your Laptop
```

---

## 3-Step Setup

### 1. Find Router (2 min)

```bash
cd /home/aaronjet/Tank_projects
./scripts/network/find_router.sh
```

### 2. Configure Router (5 min)

```bash
# Access router on default network
sudo ./scripts/network/access_xiaomi_default.sh
```

**In router web interface, configure:**

1. **LAN IP:** `192.168.2.1`
2. **Enable DHCP:**
   - Range: `192.168.2.150-200`
   - Reserve: .100, .62, .63
3. **WiFi:**
   - SSID: `Tank_Rover`
   - Password: [your choice]
4. **Save & Reboot**

### 3. Connect & Test (2 min)

**WiFi:**
```bash
# Connect to: Tank_Rover
# You'll get IP: 192.168.2.150-200 (auto)
ping 192.168.2.100
ssh aaronjet@192.168.2.100
```

**Or plug ethernet directly into switch:**
```bash
# Set laptop IP: 192.168.2.50/24
ping 192.168.2.100
ssh aaronjet@192.168.2.100
```

---

## IP Addresses

| Device | IP | How |
|--------|-----|-----|
| Jetson | 192.168.2.100 | Already set |
| Router | 192.168.2.1 | You configure |
| LiDAR Front | 192.168.2.62 | Already set |
| LiDAR Rear | 192.168.2.63 | Already set |
| Your Laptop | 192.168.2.X | DHCP or manual |

---

## Router Settings Summary

**Must Configure:**
- ✅ Router IP: `192.168.2.1`
- ✅ DHCP: Enabled (range 150-200)
- ✅ WiFi: `Tank_Rover` with password

**Don't Need:**
- ❌ Internet/WAN settings (ignore)
- ❌ Gateway (just local network)
- ❌ DNS servers (no internet)
- ❌ Port forwarding
- ❌ VPN

---

## Quick Commands

```bash
# Find router
./scripts/network/find_router.sh

# Access router (if needed)
sudo ./scripts/network/access_xiaomi_default.sh

# After setup, access router settings
firefox http://192.168.2.1

# Connect from laptop
ssh aaronjet@192.168.2.100

# View ROS topics
ros2 topic list
```

---

## Troubleshooting

**Can't find router?**
```bash
sudo ./scripts/network/access_xiaomi_default.sh
```

**Can't access Jetson after WiFi connects?**
- Check laptop IP is 192.168.2.X
- Try manual IP: 192.168.2.50

**WiFi not visible?**
- Wait 2 min after router reboot
- Check router power
- Verify WiFi enabled in settings

---

## That's It!

Simple local network for rover control.

**Access router:** http://192.168.2.1  
**SSH to Jetson:** ssh aaronjet@192.168.2.100  
**WiFi:** Tank_Rover

**Full docs:** `docs/setup/ROUTER_SIMPLE_SETUP.md`

---

**Time:** ~10 minutes total

