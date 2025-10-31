#!/usr/bin/env python3
"""
Launch file for ZED-F9P GNSS
Uses aussierobots/ublox_dgnss driver
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/ttyACM0',
        description='GNSS serial device'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('tank_sensors'),
            'config',
            'gnss_f9p.yaml'
        ]),
        description='Path to GNSS config file'
    )
    
    # GNSS node
    gnss_node = Node(
        package='ublox_dgnss',
        executable='ublox_dgnss_node',
        name='gnss_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            ('fix', '/gnss/fix'),
            ('navpvt', '/gnss/navpvt'),
            ('dop', '/gnss/dop'),
        ]
    )
    
    return LaunchDescription([
        device_arg,
        config_file_arg,
        gnss_node,
    ])

