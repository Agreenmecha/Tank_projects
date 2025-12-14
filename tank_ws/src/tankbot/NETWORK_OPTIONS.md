# Network Options for Remote Visualization

## Option 1: Xiaomi Router (Rover's Local Network) ⭐ RECOMMENDED

**Best for:** Easiest setup, most reliable

### Setup

1. **Connect laptop to Xiaomi router WiFi**
2. **On both machines, set ROS_DOMAIN_ID:**
   ```bash
   export ROS_DOMAIN_ID=42
   ```
3. **Verify same subnet:**
   ```bash
   # On laptop
   ip addr show wlan0  # or your WiFi interface
   # Should show same subnet as Jetson (e.g., 192.168.1.x)
   
   # On Jetson
   ip addr show wlan0  # or Jetson's WiFi interface
   ```
4. **Test connectivity:**
   ```bash
   # From laptop
   ping <jetson_ip>
   
   # Should get responses
   ```
5. **Verify ROS2 discovery:**
   ```bash
   # On laptop (with sensors running on Jetson)
   ros2 topic list
   # Should see: /lidar_front/pointcloud, /lidar_rear/pointcloud, etc.
   ```

**That's it!** Multicast discovery works automatically on the same subnet.

---

## Option 2: House Network

**Works if:** Both devices on same subnet OR configured for unicast

### Quick Test First

1. **Connect laptop to house WiFi**
2. **Check if Jetson is reachable:**
   ```bash
   ping <jetson_ip>
   ```
3. **Check if same subnet:**
   ```bash
   # On laptop
   ip route | grep default
   ip addr show
   
   # On Jetson  
   ip route | grep default
   ip addr show
   
   # If both show same subnet (e.g., 192.168.0.x), it might work!
   ```
4. **Try ROS2 discovery:**
   ```bash
   # Set ROS_DOMAIN_ID=42 on both
   # On laptop
   ros2 topic list
   
   # If topics appear → It works! Use this setup.
   # If no topics → Need unicast configuration (see below)
   ```

### If Discovery Doesn't Work (Different Subnets)

Configure FastRTPS for unicast peer discovery:

#### Step 1: Find IP Addresses

**On Jetson:**
```bash
hostname -I
# Note the IP address (e.g., 192.168.0.50)
```

**On Laptop:**
```bash
hostname -I
# Note the IP address (e.g., 192.168.1.100)
```

#### Step 2: Create FastRTPS XML Configuration

**On Jetson (`~/fastdds_jetson.xml`):**
```xml
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="default" is_default_profile="true">
        <rtps>
            <builtin>
                <discovery_config>
                    <discoveryProtocol>SIMPLE</discoveryProtocol>
                    <use_SIMPLE_EndpointDiscoveryProtocol>true</use_SIMPLE_EndpointDiscoveryProtocol>
                    <use_STATIC_EndpointDiscoveryProtocol>false</use_STATIC_EndpointDiscoveryProtocol>
                    <staticdiscovery>
                        <participant>
                            <name>default_xmlCreatoraaronjet@192.168.0.50</name>
                            <metatrafficUnicastLocatorList>
                                <locator>
                                    <udpv4>
                                        <address>192.168.0.50</address>
                                    </udpv4>
                                </locator>
                            </metatrafficUnicastLocatorList>
                        </participant>
                        <participant>
                            <name>default_xmlCreatoraaronubuntu@192.168.1.100</name>
                            <metatrafficUnicastLocatorList>
                                <locator>
                                    <udpv4>
                                        <address>192.168.1.100</address>
                                    </udpv4>
                                </locator>
                            </metatrafficUnicastLocatorList>
                        </participant>
                    </staticdiscovery>
                </discovery_config>
            </builtin>
        </rtps>
    </participant>
</profiles>
```

**On Laptop (`~/fastdds_laptop.xml`):**
```xml
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="default" is_default_profile="true">
        <rtps>
            <builtin>
                <discovery_config>
                    <discoveryProtocol>SIMPLE</discoveryProtocol>
                    <use_SIMPLE_EndpointDiscoveryProtocol>true</use_SIMPLE_EndpointDiscoveryProtocol>
                    <use_STATIC_EndpointDiscoveryProtocol>false</use_STATIC_EndpointDiscoveryProtocol>
                    <staticdiscovery>
                        <participant>
                            <name>default_xmlCreatoraaronjet@192.168.0.50</name>
                            <metatrafficUnicastLocatorList>
                                <locator>
                                    <udpv4>
                                        <address>192.168.0.50</address>
                                    </udpv4>
                                </locator>
                            </metatrafficUnicastLocatorList>
                        </participant>
                        <participant>
                            <name>default_xmlCreatoraaronubuntu@192.168.1.100</name>
                            <metatrafficUnicastLocatorList>
                                <locator>
                                    <udpv4>
                                        <address>192.168.1.100</address>
                                    </udpv4>
                                </locator>
                            </metatrafficUnicastLocatorList>
                        </participant>
                    </staticdiscovery>
                </discovery_config>
            </builtin>
        </rtps>
    </participant>
</profiles>
```

**⚠️ Update IP addresses in XML files to match your actual IPs!**

#### Step 3: Use XML Configuration

**On Jetson:**
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds_jetson.xml
export ROS_DOMAIN_ID=42
```

**On Laptop:**
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds_laptop.xml
export ROS_DOMAIN_ID=42
```

#### Step 4: Test

```bash
# On laptop
ros2 topic list
# Should now see Jetson topics
```

---

## Recommendation

**Use Option 1 (Xiaomi Router)** because:
- ✅ Zero configuration needed
- ✅ Multicast works automatically
- ✅ Most reliable
- ✅ Lower latency
- ✅ No firewall/router configuration

**Use Option 2 (House Network)** only if:
- You need to access internet from laptop while visualizing
- Xiaomi router doesn't have internet access
- You're willing to configure unicast if needed

---

## Quick Decision Guide

**Same subnet?** → Use either network, both work  
**Different subnets?** → Use Xiaomi router OR configure unicast  
**Need simplest setup?** → Use Xiaomi router  
**Need internet access?** → Use house network (may need unicast config)

---

## Testing Network Setup

### Test 1: Connectivity
```bash
ping <jetson_ip>
# Should get responses
```

### Test 2: ROS2 Discovery
```bash
# On laptop (with ROS_DOMAIN_ID=42 set)
ros2 topic list
# Should see Jetson topics if discovery works
```

### Test 3: Topic Data
```bash
# On laptop
ros2 topic hz /lidar_front/pointcloud
# Should show ~10 Hz if everything works
```

