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
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package share directory
    tank_sensors_share = FindPackageShare('tank_sensors')
    
    # URDF file path - read at launch time
    pkg_share = get_package_share_directory('tankbot')
    urdf_path = os.path.join(pkg_share, 'urdf', 'tankbot.urdf')
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
    
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
        default_value='false',
        description='Enable camera'
    )
    
    # Robot state publisher (publishes URDF transforms)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'publish_frequency': 50.0,
        }]
    )
    
    # Static transform from world to base_footprint (RViz needs a root frame)
    static_tf_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_footprint'],
        output='screen'
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
        robot_state_publisher,
        static_tf_world,
        gnss_launch,
        lidar_launch,
        camera_launch,
    ])

