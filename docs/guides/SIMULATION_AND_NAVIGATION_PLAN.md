# Simulation & Navigation Plan for Tank Rover

**Goal:** Develop autonomous navigation (roads, sidewalks, obstacle avoidance) using Gazebo simulation  
**Sensors:** LiDAR (primary) + GNSS (localization)  
**Stack:** ROS2 Humble + Nav2 + Gazebo

---

## 📋 Overall Architecture

```
┌─────────────────────────────────────────────────────────┐
│                   Tank Navigation Stack                 │
├─────────────────────────────────────────────────────────┤
│ Nav2 Navigation Layer                                   │
│  ├─ Global Planner (path planning)                      │
│  ├─ Local Controller (obstacle avoidance)               │
│  ├─ Behavior Tree (mission logic)                       │
├─────────────────────────────────────────────────────────┤
│ Perception Layer                                         │
│  ├─ LiDAR Processing (costmap building)                 │
│  ├─ GNSS Localization (global position)                 │
│  └─ Point-LIO (sensor fusion)                           │
├─────────────────────────────────────────────────────────┤
│ Control Layer                                            │
│  ├─ ODrive Interface Node (velocity commands)           │
│  ├─ Motor Control (differential drive kinematics)       │
│  └─ Encoder Odometry (wheel feedback)                   │
├─────────────────────────────────────────────────────────┤
│ Simulation Layer (Gazebo)                               │
│  ├─ Tank URDF with physics                              │
│  ├─ Virtual LiDAR plugin                                │
│  ├─ Virtual GNSS plugin                                 │
│  └─ Simulated environment (roads, sidewalks, obstacles) │
└─────────────────────────────────────────────────────────┘
```

---

## 🔄 Development Workflow

### Phase 1: URDF & Simulation Setup (Current)
1. ✅ ODrive communication working
2. ⏳ URDF from OnShape ready
3. ⏳ Gazebo simulation configured
4. ⏳ Sensor simulators (LiDAR, GNSS)

### Phase 2: Sensor Processing
5. ⏳ LiDAR costmap generation
6. ⏳ GNSS odometry integration
7. ⏳ Point-LIO fusion (optional)

### Phase 3: Navigation
8. ⏳ Nav2 stack configuration
9. ⏳ Path planning & local costmap
10. ⏳ Obstacle avoidance behavior
11. ⏳ Autonomous waypoint navigation

### Phase 4: Testing & Refinement
12. ⏳ Sim-to-real transfer
13. ⏳ Real robot testing
14. ⏳ Parameter tuning

---

## 📦 Required Packages

Already have:
- ✅ `tank_description` (URDF)
- ✅ `tank_control` (ODrive interface)
- ✅ `tank_navigation` (Nav2)
- ✅ `tank_sensors` (sensor drivers)
- ✅ `tank_localization` (fusion)

Need to set up:
- ⏳ Gazebo plugins for sensors
- ⏳ Nav2 configuration
- ⏳ Simulation launch files

---

## 🎮 Gazebo Simulation Setup

### 1. Install Gazebo (if not already)
```bash
sudo apt install ros-humble-gazebo-ros ros-humble-gazebo-plugins -y
```

### 2. Create Gazebo World File

**File:** `tank_description/worlds/outdoor_navigation.world`

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="outdoor_navigation">
    <!-- Physics -->
    <physics name="default_physics" default="true" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Ambient Light -->
    <light name="sun" type="directional">
      <cast_shadows>1</cast_shadows>
      <intensity>1.0</intensity>
      <direction>0.3 0.3 -1</direction>
    </light>

    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Obstacles (sidewalk, curbs, etc.) -->
    <model name="obstacle_1">
      <static>true</static>
      <pose>5 0 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.2 2 0.5</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.2 2 0.5</size>
            </box>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

### 3. Add Sensors to URDF

After importing URDF from OnShape, add sensor plugins:

**In tank.urdf.xacro, add:**

```xml
<!-- Simulated LiDAR Sensor -->
<link name="front_lidar">
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
  </inertial>
</link>

<joint name="front_lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="front_lidar"/>
  <origin xyz="0.3 0 0.15" rpy="0 0 0"/>  <!-- Adjust to your actual position -->
</joint>

<!-- LiDAR Gazebo Plugin -->
<gazebo reference="front_lidar">
  <sensor name="front_lidar" type="ray">
    <pose>0 0 0 0 0 0</pose>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1.0</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.08</min>
        <max>30.0</max>
        <resolution>0.02</resolution>
      </range>
    </ray>
    <always_on>1</always_on>
    <update_rate>10</update_rate>
    <visualize>true</visualize>
    <plugin name="gazebo_ros_ray_sensor" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <remapping>~/out:=scan</remapping>  <!-- Publishes to /scan topic -->
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>front_lidar</frame_name>
    </plugin>
  </sensor>
</gazebo>

<!-- GNSS/GPS Sensor (Gazebo) -->
<link name="gnss">
  <inertial>
    <mass value="0.05"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>

<joint name="gnss_joint" type="fixed">
  <parent link="base_link"/>
  <child link="gnss"/>
  <origin xyz="0 0 0.3" rpy="0 0 0"/>  <!-- Antenna height -->
</joint>

<!-- GNSS Gazebo Plugin -->
<gazebo reference="gnss">
  <sensor name="gnss" type="gps">
    <always_on>1</always_on>
    <update_rate>10</update_rate>
    <plugin name="gazebo_ros_gps_sensor" filename="libgazebo_ros_gps_sensor.so">
      <ros>
        <remapping>~/out:=fix</remapping>  <!-- Publishes to /fix topic -->
      </ros>
      <frame_name>gnss</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

---

## 🗺️ Nav2 Configuration

### 1. Create Nav2 Config File

**File:** `tank_navigation/config/nav2_params.yaml`

```yaml
amcl:
  ros__parameters:
    use_sim_time: true
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_search_increment: 0.05
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: 0.1
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_z_good: 0.9
    pf_z_max: 0.95
    pf_z_rand: 0.05
    pf_z_short: 0.05
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "differential"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    sigma_z: 0.05
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

bt_navigator:
  ros__parameters:
    use_sim_time: true
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odometry/filtered
    bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
    default_nav_through_poses_bt_xml: "navigate_through_poses_w_replanning_and_recovery.xml"
    default_nav_to_pose_bt_xml: "navigate_to_pose_w_replanning_and_recovery.xml"
    default_server_timeout: 20
    transform_tolerance: 0.1
    use_sim_time: true

controller_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_core/StationaryGoalChecker"
      stationary_trans_dist: 0.05
      stationary_rot_dist: 0.04
      trans_stopped_velocity: 0.025
      rot_stopped_velocity: 0.025

    general_goal_checker:
      stationary_trans_dist: 0.05
      stationary_rot_dist: 0.04
      trans_stopped_velocity: 0.025
      rot_stopped_velocity: 0.025

    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.5
      lookahead_dist: 0.6
      min_approach_linear_vel: 0.05
      approach_vel_scaling_dist: 1.0
      max_allowed_time_error: 1.0
      use_velocity_scaled_lookahead_dist: false
      min_turning_radius: 0.2
      angular_vel_max: 2.0
      max_robot_pose_search_dist: 2.0
      use_collision_detection: true
      transform_tolerance: 0.1

costmap_converter:
  ros__parameters:
    use_sim_time: true

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: true
      rolling_window: true
      width: 5
      height: 5
      resolution: 0.05
      robot_radius: 0.3
      plugins: ["obstacle_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: true
      resolution: 0.05
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: true
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: false

smoother_server:
  ros__parameters:
    use_sim_time: true
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 0.3
      max_its: 1000
      do_refinement: true

behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "drive_on_heading"]
    spin:
      plugin: "nav2_behaviors::Spin"
    backup:
      plugin: "nav2_behaviors::BackUp"
    drive_on_heading:
      plugin: "nav2_behaviors::DriveOnHeading"
```

### 2. Create Launch File

**File:** `tank_navigation/launch/nav2_sim.launch.py`

```python
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    tank_description_dir = get_package_share_directory('tank_description')
    tank_nav_dir = get_package_share_directory('tank_navigation')
    
    nav2_params = os.path.join(tank_nav_dir, 'config', 'nav2_params.yaml')
    
    return LaunchDescription([
        # Start Gazebo
        Node(
            package='gazebo_ros',
            executable='gzserver',
            arguments=[os.path.join(tank_description_dir, 'worlds', 'outdoor_navigation.world')],
            output='screen',
        ),
        
        # Start Gazebo Client
        Node(
            package='gazebo_ros',
            executable='gzclient',
            output='screen',
        ),
        
        # Spawn Robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'tank',
                '-file', os.path.join(tank_description_dir, 'urdf', 'tank.urdf'),
            ],
            output='screen',
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=[os.path.join(tank_description_dir, 'urdf', 'tank.urdf')],
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        
        # AMCL Localization
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params],
            remappings=[
                ('/scan', '/scan'),
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],
        ),
        
        # Nav2 Navigation Stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    get_package_share_directory('nav2_bringup'),
                    'launch',
                    'navigation_launch.py'
                ])
            ),
            launch_arguments={'params_file': nav2_params}.items(),
        ),
    ])
```

---

## 🚀 Running Simulation

### 1. Build Workspace
```bash
cd /home/aaronjet/Tank_projects/tank_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select tank_navigation tank_description
source install/setup.bash
```

### 2. Launch Gazebo + Nav2
```bash
ros2 launch tank_navigation nav2_sim.launch.py
```

### 3. In RViz, set navigation goal
- Click "Nav2 Goal" button
- Click on map where you want robot to go
- Robot should plan path and navigate autonomously

---

## 🧭 Navigation Features

### Path Planning
- Global planner finds optimal path
- Local controller avoids obstacles
- Replanning on dynamic obstacles

### Obstacle Avoidance
- Inflated costmap around obstacles
- Regulated pure pursuit controller
- Velocity scaling near obstacles

### Sensor Fusion
- LiDAR costmap (local obstacles)
- GNSS localization (global position)
- Wheel odometry (motion tracking)

---

## 📊 Data Flow

```
LiDAR ──→ /scan ──→ Costmap Layer ──┐
                                      ├─→ Nav2 Planner ──→ Path ──→ Controller ──→ /cmd_vel
GNSS  ──→ /fix  ──→ AMCL (location) ─┤
Odometry ────────────→ Odometry TF   ──┘
```

---

## ✅ Simulation Checklist

- [ ] URDF imported from OnShape
- [ ] Gazebo world created with obstacles
- [ ] LiDAR simulation working (see /scan topic)
- [ ] GNSS simulation working (see /fix topic)
- [ ] Nav2 parameters configured
- [ ] Launch files created and tested
- [ ] Robot spawns in Gazebo correctly
- [ ] Can set navigation goals in RViz
- [ ] Robot navigates autonomously

---

## 🔄 Real Robot Transfer

Once simulation works:

1. **Replace Gazebo LiDAR** with real LiDAR driver
2. **Replace Gazebo GNSS** with real u-blox driver
3. **ODrive control** already connected (just use real USB)
4. **Same Nav2 stack** controls both simulation and real robot

**Key:** Same ROS2 topics, so sim/real are interchangeable!

---

## 📚 Reference

- Nav2 Docs: https://navigation.ros.org/
- Gazebo Plugins: https://classic.gazebosim.org/tutorials
- ROS2 URDF: https://docs.ros.org/en/humble/Tutorials/URDF/URDF-Main.html

---

## Next Steps

1. **Get URDF from OnShape** ← We're here
2. **Import to ROS2 workspace**
3. **Set up Gazebo simulation** (following this guide)
4. **Configure Nav2**
5. **Test autonomous navigation**

Ready to proceed with OnShape URDF?

