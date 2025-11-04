#!/usr/bin/env python3
"""
Complete motor control stack launch file
Launches ODrive interface + safety monitor
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    odrive_config_arg = DeclareLaunchArgument(
        'odrive_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('tank_control'),
            'config',
            'odrive_params.yaml'
        ]),
        description='Path to ODrive configuration file'
    )
    
    safety_config_arg = DeclareLaunchArgument(
        'safety_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('tank_control'),
            'config',
            'safety_limits.yaml'
        ]),
        description='Path to safety configuration file'
    )
    
    enable_safety_arg = DeclareLaunchArgument(
        'enable_safety',
        default_value='true',
        description='Enable safety monitor node'
    )
    
    # ODrive interface node
    odrive_node = Node(
        package='tank_control',
        executable='odrive_interface_node.py',
        name='odrive_interface_node',
        output='screen',
        parameters=[LaunchConfiguration('odrive_config')],
        respawn=True,
        respawn_delay=2.0,
    )
    
    # Safety monitor node
    safety_node = Node(
        package='tank_control',
        executable='safety_monitor_node.py',
        name='safety_monitor_node',
        output='screen',
        parameters=[LaunchConfiguration('safety_config')],
        condition=LaunchConfiguration('enable_safety'),
    )
    
    return LaunchDescription([
        odrive_config_arg,
        safety_config_arg,
        enable_safety_arg,
        odrive_node,
        safety_node,
    ])

