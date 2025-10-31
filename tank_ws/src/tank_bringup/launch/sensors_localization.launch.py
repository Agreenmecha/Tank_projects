#!/usr/bin/env python3
"""
Master launch file for tank sensors + localization
Brings up:
- Dual Unitree L2 LiDARs
- ZED-F9P GNSS
- e-CAM25 Camera
- Point-LIO (using front L2)

Usage:
  ros2 launch tank_bringup sensors_localization.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    enable_gnss_arg = DeclareLaunchArgument(
        'enable_gnss',
        default_value='true',
        description='Enable GNSS sensor'
    )
    
    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='true',
        description='Enable camera'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz'
    )
    
    # Include hardware sensors
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tank_sensors'),
                'launch',
                'hardware.launch.py'
            ])
        ]),
        launch_arguments={
            'enable_gnss': LaunchConfiguration('enable_gnss'),
            'enable_lidars': 'true',  # Always enable for Point-LIO
            'enable_camera': LaunchConfiguration('enable_camera'),
        }.items()
    )
    
    # Include Point-LIO localization
    point_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tank_localization'),
                'launch',
                'point_lio.launch.py'
            ])
        ]),
        launch_arguments={
            'rviz': LaunchConfiguration('rviz'),
        }.items()
    )
    
    return LaunchDescription([
        enable_gnss_arg,
        enable_camera_arg,
        rviz_arg,
        hardware_launch,
        point_lio_launch,
    ])

