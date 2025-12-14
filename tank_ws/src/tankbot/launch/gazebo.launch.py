#!/usr/bin/env python3
"""
Launch Gazebo with tankbot URDF model.
Exported from SolidWorks URDF exporter.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('tankbot')
    
    # URDF file path
    urdf_path = os.path.join(pkg_share, 'urdf', 'tankbot.urdf')
    
    # Read URDF file
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
    
    # Include Gazebo empty world launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ])
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': 'true'
        }]
    )
    
    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_tankbot',
        output='screen',
        arguments=[
            '-entity', 'tankbot',
            '-file', urdf_path,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ]
    )
    
    # Static transform publisher for base_footprint (common ROS convention)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        static_tf,
        spawn_entity,
    ])

