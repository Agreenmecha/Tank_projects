#!/usr/bin/env python3
"""
GPS Waypoint Navigation Launch
Complete system for GPS-based autonomous navigation
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launch complete GPS waypoint navigation system:
    1. LiDAR + Point-LIO (localization)
    2. Navsat Transform (GPS → map coordinates)
    3. Nav2 (navigation)
    4. GPS Waypoint Manager (mission control)
    5. ROSBridge (for web interface)
    """
    
    tank_nav_dir = get_package_share_directory('tank_navigation')
    
    # Config files
    navsat_config = os.path.join(tank_nav_dir, 'config', 'gps', 'navsat_transform.yaml')
    
    # Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    start_nav2_arg = DeclareLaunchArgument(
        'start_nav2',
        default_value='true',
        description='Start Nav2 navigation stack'
    )
    
    start_rosbridge_arg = DeclareLaunchArgument(
        'start_rosbridge',
        default_value='true',
        description='Start ROSBridge for web interface'
    )
    
    # 1. Navsat Transform Node (GPS → Map coordinates)
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[navsat_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('imu', '/lidar_front/imu'),
            ('gps/fix', '/ubx_nav_hp_pos_llh'),
            ('odometry/filtered', '/Odometry'),
        ]
    )
    
    # 2. GPS Waypoint Manager
    gps_waypoint_manager = Node(
        package='tank_navigation',
        executable='gps_waypoint_manager',
        name='gps_waypoint_manager',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # 3. Nav2 (optional, if not already running)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tank_navigation'),
                'launch',
                'navigation.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('start_nav2')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )
    
    # 4. ROSBridge for web interface
    rosbridge_server = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': 9090,
            'address': '0.0.0.0'  # Allow external connections
        }],
        condition=IfCondition(LaunchConfiguration('start_rosbridge'))
    )
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        start_nav2_arg,
        start_rosbridge_arg,
        
        # Nodes
        navsat_transform_node,
        gps_waypoint_manager,
        nav2_launch,
        rosbridge_server,
    ])

