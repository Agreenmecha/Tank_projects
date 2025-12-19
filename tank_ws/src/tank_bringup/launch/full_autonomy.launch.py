#!/usr/bin/env python3
"""
Tank Full Autonomy Launch File
================================

Brings up the COMPLETE autonomous navigation stack:
  1. Robot Description (URDF + TF tree)
  2. Sensors (Dual LiDAR + GPS)
  3. Localization (Point-LIO)
  4. Navigation (Nav2)
  5. Motor Control (ODrive)

Usage:
  Basic autonomous navigation (no GPS):
    ros2 launch tank_bringup full_autonomy.launch.py
    
  GPS waypoint missions:
    ros2 launch tank_bringup full_autonomy.launch.py enable_gps_waypoints:=true
    
  With RViz visualization:
    ros2 launch tank_bringup full_autonomy.launch.py rviz:=true
    
  Disable motor control (testing only):
    ros2 launch tank_bringup full_autonomy.launch.py enable_motors:=false
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    LogInfo
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate complete autonomous navigation launch description"""
    
    # =================================================================
    # ARGUMENTS
    # =================================================================
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (for Gazebo/bag playback)'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2 for visualization (use remote RViz on laptop instead)'
    )
    
    enable_gps_arg = DeclareLaunchArgument(
        'enable_gps',
        default_value='true',
        description='Enable GPS sensor'
    )
    
    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='false',
        description='Enable camera (e-CAM25)'
    )
    
    enable_motors_arg = DeclareLaunchArgument(
        'enable_motors',
        default_value='true',
        description='Enable motor control (set false for testing without hardware)'
    )
    
    enable_gps_waypoints_arg = DeclareLaunchArgument(
        'enable_gps_waypoints',
        default_value='false',
        description='Enable GPS waypoint navigation system (includes ROSBridge)'
    )
    
    nav2_params_arg = DeclareLaunchArgument(
        'nav2_params',
        default_value='nav2_params.yaml',
        description='Nav2 params file (nav2_params.yaml or nav2_params_gps.yaml)'
    )
    
    # =================================================================
    # 1. ROBOT DESCRIPTION (URDF + TF)
    # =================================================================
    
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tank_description'),
                'launch',
                'robot_state_publisher.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )
    
    # =================================================================
    # 2. SENSORS (LiDAR + GPS + Camera)
    # =================================================================
    
    # Dual LiDAR launch (always enabled for navigation)
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tank_sensors'),
                'launch',
                'lidar_dual.launch.py'
            ])
        ])
    )
    
    # GPS launch (conditional)
    gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tank_sensors'),
                'launch',
                'gnss.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('enable_gps'))
    )
    
    # Camera launch (conditional, optional)
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tank_sensors'),
                'launch',
                'camera_argus.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('enable_camera'))
    )
    
    # =================================================================
    # 3. LOCALIZATION (Point-LIO)
    # =================================================================
    
    point_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tank_localization'),
                'launch',
                'point_lio.launch.py'
            ])
        ]),
        launch_arguments={
            'rviz': 'false',  # RViz will be launched separately if requested
        }.items()
    )
    
    # =================================================================
    # 4. NAVIGATION (Nav2)
    # =================================================================
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tank_navigation'),
                'launch',
                'navigation.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': PathJoinSubstitution([
                FindPackageShare('tank_navigation'),
                'config',
                'nav2',
                LaunchConfiguration('nav2_params')
            ]),
            'autostart': 'true',
        }.items()
    )
    
    # =================================================================
    # 5. MOTOR CONTROL (ODrive)
    # =================================================================
    
    motor_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tank_control'),
                'launch',
                'motor_control.launch.py'  # Includes ODrive + safety monitor
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('enable_motors'))
    )
    
    # =================================================================
    # 6. GPS WAYPOINT SYSTEM (Optional)
    # =================================================================
    
    # Navsat transform (GPS → map coordinates)
    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('tank_navigation'),
            'config',
            'gps',
            'navsat_transform.yaml'
        ])],
        remappings=[
            ('gps/fix', '/gnss/fix'),  # From ublox_dgnss
            ('imu/data', '/lidar_front/imu'),
            ('odometry/filtered', '/Odometry')  # From Point-LIO
        ],
        condition=IfCondition(LaunchConfiguration('enable_gps_waypoints'))
    )
    
    # GPS Waypoint Manager
    gps_waypoint_manager = Node(
        package='tank_navigation',
        executable='gps_waypoint_manager.py',
        name='gps_waypoint_manager',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        condition=IfCondition(LaunchConfiguration('enable_gps_waypoints'))
    )
    
    # ROSBridge for web interface
    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_gps_waypoints'))
    )
    
    # =================================================================
    # 7. VISUALIZATION (RViz)
    # =================================================================
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('tank_navigation'),
            'rviz',
            'nav2_default_view.rviz'
        ])],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    # =================================================================
    # LAUNCH DESCRIPTION
    # =================================================================
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        rviz_arg,
        enable_gps_arg,
        enable_camera_arg,
        enable_motors_arg,
        enable_gps_waypoints_arg,
        nav2_params_arg,
        
        # Startup message
        LogInfo(msg='========================================'),
        LogInfo(msg='Tank Full Autonomy System Starting...'),
        LogInfo(msg='========================================'),
        
        # 1. Robot Description
        LogInfo(msg='[1/7] Starting Robot Description...'),
        robot_description,
        
        # 2. Sensors
        LogInfo(msg='[2/7] Starting Sensors (LiDAR, GPS, Camera)...'),
        lidar_launch,
        gps_launch,
        camera_launch,
        
        # 3. Localization
        LogInfo(msg='[3/7] Starting Localization (Point-LIO)...'),
        point_lio_launch,
        
        # 4. Navigation
        LogInfo(msg='[4/7] Starting Navigation (Nav2)...'),
        nav2_launch,
        
        # 5. Motor Control
        LogInfo(msg='[5/7] Starting Motor Control (ODrive)...'),
        motor_control_launch,
        
        # 6. GPS Waypoint System
        LogInfo(msg='[6/7] Starting GPS Waypoint System (if enabled)...'),
        navsat_transform,
        gps_waypoint_manager,
        rosbridge,
        
        # 7. Visualization
        LogInfo(msg='[7/7] Starting Visualization (RViz)...'),
        rviz_node,
        
        LogInfo(msg='========================================'),
        LogInfo(msg='Tank Autonomy System Ready!'),
        LogInfo(msg='========================================'),
    ])

