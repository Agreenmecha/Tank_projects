#!/usr/bin/env python3
"""
Full Tank Navigation System Launch
Launches complete autonomous navigation stack:
- LiDAR drivers (dual L2)
- Point-LIO (localization)
- Nav2 (navigation)
- Robot state publisher
- RViz (optional)
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
    """Launch complete autonomous navigation system"""
    
    # Arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # 1. Launch LiDAR drivers (dual L2)
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tank_sensors'),
                'launch',
                'lidar_dual.launch.py'
            ])
        ])
    )
    
    # 2. Launch Point-LIO (localization)
    point_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tank_localization'),
                'launch',
                'point_lio.launch.py'
            ])
        ]),
        launch_arguments={
            'rviz': 'false',  # Don't launch Point-LIO's RViz
        }.items()
    )
    
    # 3. Launch Nav2 (navigation)
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
        }.items()
    )
    
    # 4. RViz with Nav2 config (optional)
    rviz_config = os.path.join(
        get_package_share_directory('tank_navigation'),
        'rviz',
        'nav2_default_view.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    return LaunchDescription([
        # Arguments
        rviz_arg,
        use_sim_time_arg,
        
        # Launch stack
        lidar_launch,
        point_lio_launch,
        nav2_launch,
        rviz_node,
    ])

