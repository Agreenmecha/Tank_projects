#!/usr/bin/env python3
"""
Launch file for all tank hardware sensors:
- Dual Unitree L2 LiDARs (front + rear)
- ZED-F9P GNSS
- e-CAM25_CUONX camera (global shutter)
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # Package share directory
    tank_sensors_share = FindPackageShare('tank_sensors')
    
    # Declare arguments
    enable_gnss_arg = DeclareLaunchArgument(
        'enable_gnss',
        default_value='true',
        description='Enable GNSS sensor'
    )
    
    enable_lidars_arg = DeclareLaunchArgument(
        'enable_lidars',
        default_value='true',
        description='Enable dual LiDARs'
    )
    
    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='true',
        description='Enable camera'
    )
    
    # Include GNSS launch
    gnss_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                tank_sensors_share,
                'launch',
                'gnss.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('enable_gnss'))
    )
    
    # Include dual LiDAR launch
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                tank_sensors_share,
                'launch',
                'lidar_dual.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('enable_lidars'))
    )
    
    # Camera launch (e-CAM25_CUONX - AR0234 global shutter)
    # Uses nvarguscamerasrc via gscam
    # NOTE: Requires e-con Systems drivers installed!
    # See: tank_ws/src/external/ECAM25_CAMERA_SETUP.md
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                tank_sensors_share,
                'launch',
                'camera_argus.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('enable_camera'))
    )
    
    return LaunchDescription([
        enable_gnss_arg,
        enable_lidars_arg,
        enable_camera_arg,
        gnss_launch,
        lidar_launch,
        camera_launch,
    ])

