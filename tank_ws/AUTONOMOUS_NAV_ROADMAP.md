# Autonomous Navigation Roadmap

## Current Status: ✅ Localization Complete

You have:
- ✅ **Sensors**: Dual L2 LiDARs, GNSS (u-blox + NTRIP), IMU
- ✅ **Localization**: Point-LIO (LiDAR odometry, 10 Hz, map→base_link)
- ✅ **Control**: ODrive motor interface, teleop
- ✅ **Robot Description**: URDF with transforms
- ✅ **Infrastructure**: ROS2 packages, launch files

---

## Phase 2: Navigation Stack Setup

### 1. ⏳ Nav2 Integration (PRIORITY)

**What**: ROS2 Navigation stack - the core autonomous navigation system

**Components Needed**:
```yaml
Nav2 Stack:
  - Controller Server (path following)
  - Planner Server (global path planning)
  - Behavior Server (actions: spin, backup, wait)
  - BT Navigator (behavior trees for decision making)
  - Waypoint Follower (multi-waypoint missions)
  - Lifecycle Manager (system coordination)
```

**Files to Create**:
- `tank_navigation/config/nav2_params.yaml`
- `tank_navigation/launch/navigation.launch.py`
- `tank_navigation/config/bt_navigator.xml` (behavior trees)

**Key Parameters to Configure**:
```yaml
Controller:
  - controller_frequency: 20.0  # Path following rate
  - Robot footprint: Track dimensions
  - Max velocities: Based on your ODrive limits
  
Planner:
  - planner_frequency: 1.0
  - Algorithm: NavFn or Smac (for differential/skid-steer)
  
Costmaps:
  - Local costmap: 5m radius for obstacle avoidance
  - Global costmap: Full map for planning
```

---

### 2. ⏳ Costmap Configuration

**What**: 2D occupancy grids for obstacle detection and path planning

**Types**:
- **Global Costmap**: Static map + known obstacles
- **Local Costmap**: Dynamic obstacles from LiDAR

**Layers to Configure**:
```yaml
Costmap Layers:
  1. Static Layer: Pre-built map (optional)
  2. Obstacle Layer: LiDAR point clouds
  3. Inflation Layer: Safety buffer around obstacles
  4. Voxel Layer: 3D obstacle handling (for slopes)
```

**Configuration File**:
- `tank_navigation/config/costmap_common.yaml`
- `tank_navigation/config/local_costmap.yaml`
- `tank_navigation/config/global_costmap.yaml`

**Point Cloud Processing**:
```yaml
Obstacle Layer:
  observation_sources: "lidar_front lidar_rear"
  lidar_front:
    topic: /lidar_front/cloud_fixed
    data_type: PointCloud2
    marking: true
    clearing: true
    min_obstacle_height: 0.1
    max_obstacle_height: 2.0
```

---

### 3. ⏳ Sensor Fusion (RECOMMENDED)

**What**: Fuse Point-LIO + GNSS + IMU for robust localization

**Package**: `robot_localization` (Extended Kalman Filter)

**Why**:
- Improves accuracy in GPS-available areas
- Provides fallback when LiDAR features are sparse
- Smooths odometry estimates

**Files to Create**:
- `tank_localization/config/ekf_config.yaml`
- `tank_localization/launch/ekf_fusion.launch.py`

**Sensor Fusion Setup**:
```yaml
EKF Inputs:
  - odom0: /Odometry (Point-LIO)
  - imu0: /lidar_front/imu (IMU data)
  - odom1: /gnss/odometry (GPS converted to odom)
  
EKF Output:
  - /odometry/filtered (fused estimate)
  - TF: odom → base_link
```

**Frame Tree**:
```
Current: map → base_link (Point-LIO)

With EKF: map → odom → base_link
          ↑       ↑
    Point-LIO   EKF
```

---

### 4. ⏳ Path Planning Algorithms

**What**: Global and local path planners

**Options**:

**Global Planners** (long-distance paths):
- **NavFn**: A* algorithm (default, good for most cases)
- **Smac Planner**: State lattice (better for tracked vehicles)
- **Theta Star**: Any-angle paths (smoother)

**Local Planners** (obstacle avoidance):
- **DWB Controller**: Dynamic Window Approach (recommended)
- **TEB Controller**: Timed Elastic Band (for smooth tracking)
- **Regulated Pure Pursuit**: Simple geometric controller

**Recommendation for Tracked Tank**:
```yaml
Global: Smac Planner (handles tracked vehicle kinematics)
Local: DWB Controller (robust obstacle avoidance)
```

---

### 5. ⏳ Behavior Trees & Recovery

**What**: Decision-making logic and recovery behaviors

**Default Nav2 Behaviors**:
- **Navigate to Pose**: Go to goal
- **Spin**: Rotate in place
- **Backup**: Reverse
- **Wait**: Pause
- **Clear Costmap**: Reset obstacle memory

**Recovery Behaviors** (when stuck):
1. Clear local costmap
2. Spin 360° to look for path
3. Backup slowly
4. Clear global costmap
5. Wait and retry

**Custom Behaviors to Add**:
- **Steep Slope Mode**: Slower, more cautious on slopes
- **Emergency Stop**: Detect tip risk
- **Multi-LiDAR Fusion**: Switch between front/rear LiDARs

---

### 6. ⏳ Velocity Smoother & Controller Tuning

**What**: Smooth velocity commands for tracked vehicle

**Why Needed**:
- Tanks can't turn instantly (track slippage)
- High acceleration = loss of traction
- Need gradual velocity changes

**Configuration**:
```yaml
Velocity Smoother:
  max_velocity: [0.5, 0.0, 0.5]  # [linear_x, linear_y, angular_z]
  max_accel: [0.3, 0.0, 0.3]
  max_decel: [-0.5, 0.0, -0.5]
  
Controller Gains:
  # Tune based on tank performance
  xy_goal_tolerance: 0.25  # meters
  yaw_goal_tolerance: 0.17  # ~10 degrees
```

---

### 7. ⏳ Safety Systems

**Critical Safety Features**:

**A. Collision Detection**:
```yaml
Features:
  - Emergency stop on imminent collision
  - Proximity zones (stop, slow, caution)
  - 360° coverage from dual LiDARs
```

**B. Tip Detection** (IMPORTANT for tracked vehicle):
```yaml
Monitor:
  - IMU pitch/roll angles
  - Detect steep slopes (>30°)
  - Stop if tip risk detected
  
Implementation:
  - tank_control/safety_monitor_node.py (already exists!)
  - Add slope monitoring
  - Integrate with Nav2
```

**C. Watchdog & Heartbeat**:
```yaml
Monitor:
  - Nav2 health
  - Sensor status
  - Motor controller connection
  - Emergency stop button
```

---

### 8. ⏳ Map Management

**Options**:

**A. No Map (Recommended Start)**:
- Use local costmap only
- Reactive navigation
- Good for exploration

**B. Pre-Built Map**:
- Use Point-LIO to create map
- Save as PCD or OccupancyGrid
- Load for repeatable missions

**C. SLAM** (Optional):
- Real-time mapping while navigating
- Use SLAM Toolbox or Cartographer
- More complex, but useful for unknown environments

**Quick Start**: Use no map, local costmap only for first tests.

---

### 9. ⏳ Waypoint Following & Missions

**What**: Navigate through multiple waypoints

**Implementation**:
```yaml
Nav2 Waypoint Follower:
  - Define GPS waypoints or map coordinates
  - Sequential or closest-first execution
  - Actions at waypoints (stop, capture image, etc.)
```

**Files to Create**:
- `tank_navigation/missions/test_waypoints.yaml`
- `tank_navigation/scripts/waypoint_loader.py`

---

### 10. ⏳ Visualization & Monitoring

**Tools Needed**:

**A. RViz Configuration**:
```yaml
Display:
  - Robot model
  - TF tree
  - Point clouds (both LiDARs)
  - Costmaps (local + global)
  - Planned path
  - Robot trajectory
  - Goal markers
```

**B. Nav2 Panel**:
- Set navigation goals in RViz
- Monitor navigation status
- Cancel/pause missions

**C. Diagnostics**:
- Topic frequencies
- Transform delays
- CPU/memory usage

---

## Implementation Priority

### Phase 2A: Basic Navigation (1-2 weeks)
1. ✅ Install Nav2: `sudo apt install ros-humble-nav2-*`
2. ⏳ Create nav2_params.yaml (start with defaults)
3. ⏳ Configure costmaps (use LiDAR data)
4. ⏳ Set up DWB controller
5. ⏳ Test in simulation (optional) or carefully in real world

### Phase 2B: Tuning & Safety (1 week)
6. ⏳ Tune controller parameters
7. ⏳ Add velocity smoother
8. ⏳ Implement safety systems
9. ⏳ Add recovery behaviors

### Phase 2C: Advanced Features (ongoing)
10. ⏳ Sensor fusion (EKF)
11. ⏳ Waypoint missions
12. ⏳ Map building/saving
13. ⏳ Behavior tree customization

---

## Quick Start: Minimal Nav2 Setup

**What you need RIGHT NOW to start testing**:

```bash
# 1. Install Nav2
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# 2. Create basic config (I can help with this)
tank_navigation/
├── config/
│   └── nav2_params.yaml      # Start with Nav2 defaults
├── launch/
│   └── navigation.launch.py  # Launch Nav2 stack
└── rviz/
    └── nav2_config.rviz      # Visualization

# 3. Launch full system
ros2 launch tank_bringup full_autonomy.launch.py

# 4. Set goal in RViz and test!
```

---

## Estimated Timeline

| Phase | Duration | Effort |
|-------|----------|--------|
| Nav2 Basic Setup | 2-3 days | Medium |
| Costmap Config | 1-2 days | Medium |
| Controller Tuning | 3-5 days | High (trial & error) |
| Safety Systems | 2-3 days | Medium |
| Sensor Fusion | 1-2 days | Low (if optional) |
| Testing & Refinement | Ongoing | High |

**Total to first autonomous waypoint**: ~2 weeks of focused work

---

## Next Steps

### Immediate (This Week):
1. Install Nav2 packages
2. Create `tank_navigation` package structure
3. Start with default Nav2 config
4. Test basic "move to pose" in safe area

### Short-term (Next Week):
5. Tune controller for tracked vehicle
6. Configure costmaps with dual LiDAR
7. Add safety monitoring
8. Test waypoint following

### Medium-term (Next Month):
9. Add sensor fusion
10. Build/save maps
11. Implement complex behaviors
12. Field testing and refinement

---

## Resources

### Nav2 Documentation
- [Nav2 Docs](https://navigation.ros.org/)
- [Configuration Guide](https://navigation.ros.org/configuration/index.html)
- [Tuning Guide](https://navigation.ros.org/tuning/index.html)

### Tutorials
- [Nav2 First-Time Setup](https://navigation.ros.org/setup_guides/index.html)
- [Tracked Vehicle Config](https://navigation.ros.org/configuration/packages/configuring-smac-planner.html)

### Similar Projects
- Clearpath Jackal/Husky (differential drive examples)
- TurtleBot4 (Nav2 reference)

---

**Current Status**: 40% complete (sensors + localization done!)  
**Next Milestone**: Nav2 basic setup  
**End Goal**: Fully autonomous waypoint navigation

---

Want me to start with Nav2 setup? I can create the basic navigation package structure and config files.

