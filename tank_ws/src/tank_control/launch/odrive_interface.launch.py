#!/usr/bin/env python3
"""
Launch file for ODrive USB interface
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('tank_control'),
            'config',
            'odrive_params.yaml'
        ]),
        description='Path to ODrive configuration file'
    )
    
    # ODrive interface node
    odrive_node = Node(
        package='tank_control',
        executable='odrive_interface_node',
        name='odrive_interface_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        respawn=True,
        respawn_delay=2.0,
    )
    
    return LaunchDescription([
        config_file_arg,
        odrive_node,
    ])

