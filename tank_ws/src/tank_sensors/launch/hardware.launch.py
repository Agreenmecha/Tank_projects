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
    
    # Camera node (e-CAM25_CUONX - AR0234 global shutter)
    # Using v4l2_camera package for simple USB camera interface
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [1280, 720],
            'camera_frame_id': 'camera_link',
            'pixel_format': 'UYVY',  # e-CAM25 outputs UYVY
            'framerate': 20.0,
            'camera_name': 'ecam25_cuonx',
        }],
        remappings=[
            ('image_raw', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ],
        condition=IfCondition(LaunchConfiguration('enable_camera'))
    )
    
    return LaunchDescription([
        enable_gnss_arg,
        enable_lidars_arg,
        enable_camera_arg,
        gnss_launch,
        lidar_launch,
        camera_node,
    ])

