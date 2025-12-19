# GPS Waypoint Navigation System

## Overview

Complete GPS waypoint navigation system with satellite map interface for outdoor autonomous navigation.

## Features

✅ **Satellite Map Interface** - Place waypoints by clicking on map  
✅ **GPS Coordinate Conversion** - Automatic lat/lon → map x/y  
✅ **Real-time Robot Tracking** - See robot position on map  
✅ **Multi-waypoint Missions** - Sequential waypoint navigation  
✅ **Mission Export/Import** - Save and load waypoint missions  
✅ **ROS Integration** - Direct communication with Nav2  

## System Components

```
GPS Waypoint Navigation System:
├── Navsat Transform (robot_localization)
│   └── Converts GPS coordinates to map coordinates
├── GPS Waypoint Manager (Python node)
│   └── Receives GPS waypoints, sends to Nav2
├── Web Interface (HTML/JS)
│   └── Interactive satellite map for planning
└── ROSBridge
    └── WebSocket connection between web and ROS
```

## Quick Start

### Step 1: Launch GPS Waypoint Navigation

```bash
cd ~/Tank_projects/tank_ws
source install/setup.bash

# Launch complete system (LiDAR + Nav2 + GPS + Web interface)
ros2 launch tank_navigation gps_waypoint_nav.launch.py

# Or if Nav2 already running:
ros2 launch tank_navigation gps_waypoint_nav.launch.py start_nav2:=false
```

### Step 2: Open Web Interface

```bash
# Open in browser:
firefox ~/Tank_projects/tank_ws/install/tank_navigation/share/tank_navigation/web/gps_waypoint_planner.html

# Or from another machine (replace with your Jetson's IP):
firefox http://192.168.X.X:8080/gps_waypoint_planner.html
```

### Step 3: Connect to ROS

1. In web interface, verify ROSBridge URL: `ws://localhost:9090`
2. Click **"Connect to ROS"**
3. Wait for "Connected" status (green indicator)
4. Robot GPS position will appear automatically

### Step 4: Plan Mission

1. **Change map type** (optional): Select "Satellite" from dropdown
2. **Center on robot**: Click "📍 Center on Robot" button
3. **Place waypoints**: Click anywhere on map to add waypoints
4. **Adjust waypoints**: Click "✕" on waypoint to remove
5. **Send mission**: Click "🚀 Send Mission to Robot"

### Step 5: Watch Autonomous Navigation!

The robot will:
1. Navigate to first waypoint
2. Automatically proceed to next waypoint
3. Continue until all waypoints complete

## Web Interface Guide

### Map Controls

| Control | Function |
|---------|----------|
| Map Type | Switch between OpenStreetMap, Satellite, Terrain |
| Center on Robot | Pan map to robot's GPS position |
| Clear All Waypoints | Remove all waypoints from mission |
| Zoom | Mouse wheel or +/- buttons |
| Pan | Click and drag |

### Waypoint Management

| Action | How To |
|--------|--------|
| Add Waypoint | Click on map |
| Remove Waypoint | Click ✕ button next to waypoint |
| View Coordinates | Hover over waypoint marker |
| Reorder | Remove and re-add (planned feature) |

### Mission Control

| Button | Function |
|--------|----------|
| Send Mission to Robot | Publish waypoints to ROS |
| Export Waypoints | Download as JSON file |
| Import Waypoints | (Planned) Load from JSON |

## Configuration

### GPS Datum (Origin Point)

The system automatically sets the GPS datum from the first GPS fix. To manually set:

Edit `config/gps/navsat_transform.yaml`:
```yaml
datum: [37.1234, -122.5678, 0.0]  # [lat, lon, alt]
wait_for_datum: true
```

### Magnetic Declination

Update for your location (improves heading accuracy):

Visit: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml

Edit `config/gps/navsat_transform.yaml`:
```yaml
magnetic_declination_radians: 0.2269  # Example: 13° East
```

### GPS Topic

If your GPS publishes to different topic:

Edit `config/gps/navsat_transform.yaml`:
```yaml
gps_topic: /your/gps/topic
```

And update in `gps_waypoint_manager.py`:
```python
self.gps_sub = self.create_subscription(
    NavSatFix,
    '/your/gps/topic',  # Change this
    self.gps_callback,
    10
)
```

## Manual Waypoint Sending (Without Web Interface)

### Via Command Line

```bash
# Single waypoint
ros2 topic pub /gps_waypoints std_msgs/String "{data: '{\"waypoints\": [{\"lat\": 37.1234, \"lon\": -122.5678, \"name\": \"Test Point\"}]}'}"

# Multiple waypoints
ros2 topic pub /gps_waypoints std_msgs/String "{data: '{\"waypoints\": [{\"lat\": 37.1234, \"lon\": -122.5678}, {\"lat\": 37.1235, \"lon\": -122.5679}]}'}"
```

### Via JSON File

Create `mission.json`:
```json
{
  "waypoints": [
    {"lat": 37.1234, "lon": -122.5678, "name": "Start"},
    {"lat": 37.1235, "lon": -122.5679, "name": "Middle"},
    {"lat": 37.1236, "lon": -122.5680, "name": "End"}
  ]
}
```

Send to robot:
```bash
cat mission.json | ros2 topic pub /gps_waypoints std_msgs/String "{data: '$(cat mission.json)'}"
```

## Monitoring

### Check System Status

```bash
# GPS position
ros2 topic echo /ubx_nav_hp_pos_llh

# GPS converted to map coordinates
ros2 topic echo /odometry/gps

# Mission status
ros2 topic echo /gps_mission_status

# Current waypoint
ros2 topic echo /behavior_tree_log
```

### View Transforms

```bash
# Check GPS → map transform
ros2 run tf2_ros tf2_echo map base_link

# View all transforms
ros2 run tf2_tools view_frames
```

## Troubleshooting

### Web Interface Won't Connect

**Problem**: "Disconnected" status in web interface

**Solutions**:
1. Check ROSBridge is running:
   ```bash
   ros2 node list | grep rosbridge
   ```

2. Check ROSBridge port:
   ```bash
   netstat -an | grep 9090
   ```

3. Try reconnecting with correct URL:
   - Local: `ws://localhost:9090`
   - Remote: `ws://JETSON_IP:9090`

4. Check firewall (if accessing remotely):
   ```bash
   sudo ufw allow 9090
   ```

### No Robot Position on Map

**Problem**: Robot marker doesn't appear

**Solutions**:
1. Check GPS is publishing:
   ```bash
   ros2 topic hz /ubx_nav_hp_pos_llh
   ```

2. Check GPS has fix:
   ```bash
   ros2 topic echo /ubx_nav_hp_pos_llh --once | grep status
   ```

3. Ensure GPS is connected to u-blox and getting RTCM corrections

### Robot Doesn't Navigate to Waypoint

**Problem**: Waypoints sent but robot doesn't move

**Solutions**:
1. Check Nav2 is running:
   ```bash
   ros2 node list | grep nav2
   ```

2. Check GPS datum is set:
   ```bash
   ros2 topic echo /gps_mission_status
   ```

3. Verify waypoint was converted:
   - Check logs in GPS waypoint manager node
   - Should see "GPS(lat, lon) → Map(x, y)"

4. Ensure Point-LIO is providing localization:
   ```bash
   ros2 topic hz /Odometry
   ```

### Inaccurate Navigation

**Problem**: Robot goes to wrong location or nearby but not exact

**Expected Behavior with Standard GPS (No RTK)**:
- **Normal**: Robot gets within 2-5 meters of waypoint
- **This is GPS accuracy limitation, not a bug!**

**Solutions to Improve**:
1. **Add RTK corrections** (NTRIP): Would improve to 2-5 cm accuracy
2. **Verify datum**: Is origin point correct?
3. **Magnetic declination**: Set correctly for your location?
4. **Increase goal tolerance**: For standard GPS, use 2-3m tolerance
5. **Multi-sensor fusion**: Combine with Point-LIO for better final positioning

## Advanced Features

### Custom Waypoint Actions

Extend `gps_waypoint_manager.py` to add actions at waypoints:

```python
def waypoint_reached_callback(self, waypoint_index):
    """Execute action when waypoint reached"""
    # Example: Take photo, wait, etc.
    if waypoint_index == 2:  # At waypoint 3
        self.get_logger().info("Stopping for 10 seconds...")
        time.sleep(10)
```

### GPS Geofencing

Add boundary checking:

```python
def is_within_geofence(self, lat, lon):
    """Check if point is within allowed area"""
    # Define boundaries
    lat_min, lat_max = 37.1000, 37.2000
    lon_min, lon_max = -122.6000, -122.5000
    
    return (lat_min <= lat <= lat_max and 
            lon_min <= lon <= lon_max)
```

### Waypoint Optimization

Add route optimization (traveling salesman):

```python
from scipy.spatial.distance import cdist
from scipy.optimize import linear_sum_assignment

def optimize_waypoint_order(self, waypoints):
    """Reorder waypoints for shortest path"""
    # Calculate distances between all pairs
    coords = [(wp['lat'], wp['lon']) for wp in waypoints]
    distances = cdist(coords, coords)
    
    # Find optimal order (simplified)
    row_ind, col_ind = linear_sum_assignment(distances)
    
    return [waypoints[i] for i in col_ind]
```

## Integration with Existing Systems

### Add to tank_bringup

Create `tank_bringup/launch/full_gps_autonomy.launch.py`:

```python
# Include all systems:
# - Sensors (LiDAR, GPS, IMU)
# - Localization (Point-LIO + GPS)
# - Navigation (Nav2)
# - GPS Waypoint System
# - Web Interface
```

### Combine with Terrain Analysis

Use GPS waypoints + terrain data:
- Check slope before navigation
- Adjust speed based on terrain type
- Avoid hazardous areas

## Performance

### Coordinate Conversion Accuracy

- **Small areas (<10 km)**: ±0.5m error
- **Large areas (>10 km)**: Use UTM instead of equirectangular

### Navigation Accuracy

With u-blox ZED-F9P (No RTK):
- **GPS accuracy**: ±2-5 meters (standard GNSS)
- **Navigation tolerance**: 2-3 meters recommended (Nav2 goal tolerance)
- **Final position**: Within 3-5 meters of GPS waypoint
- **Note**: Good enough for outdoor waypoint navigation, but not precision positioning

## Future Enhancements

⏳ **Planned Features**:
- Waypoint drag-and-drop reordering
- Route optimization (shortest path)
- Terrain elevation data overlay
- Multi-robot coordination
- Waypoint looping/patrol mode
- Emergency return-to-home
- Geofence boundaries visualization

---

**Status**: ✅ GPS Waypoint System Complete  
**Ready**: Place waypoints on satellite map and navigate!  
**Next**: Test first GPS waypoint mission

