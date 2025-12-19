#!/usr/bin/env python3
"""
Launch GPS waypoint follower.

Requires:
- /gnss/fix        (NavSatFix)
- /imu/fused       (Imu)
- Publishes /cmd_vel
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('tank_navigation'),
            'config',
            'gps_nav.yaml'
        ]),
        description='Path to GPS navigation config'
    )

    gps_nav_node = Node(
        package='tank_navigation',
        executable='gps_waypoint_follower.py',
        name='gps_waypoint_follower',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )

    return LaunchDescription([
        config_arg,
        gps_nav_node,
    ])

