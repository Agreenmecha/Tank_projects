#!/usr/bin/env python3
"""
Launch file for ROS2 gamepad teleop
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    max_linear_arg = DeclareLaunchArgument(
        'max_linear_vel',
        default_value='1.5',
        description='Maximum linear velocity (m/s)'
    )
    
    max_angular_arg = DeclareLaunchArgument(
        'max_angular_vel',
        default_value='2.0',
        description='Maximum angular velocity (rad/s)'
    )
    
    deadzone_arg = DeclareLaunchArgument(
        'deadzone',
        default_value='0.15',
        description='Joystick deadzone'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='50.0',
        description='Command publish rate (Hz)'
    )
    
    # Teleop node
    teleop_node = Node(
        package='tank_control',
        executable='teleop_gamepad_node.py',
        name='teleop_gamepad_node',
        output='screen',
        parameters=[{
            'max_linear_vel': LaunchConfiguration('max_linear_vel'),
            'max_angular_vel': LaunchConfiguration('max_angular_vel'),
            'deadzone': LaunchConfiguration('deadzone'),
            'publish_rate': LaunchConfiguration('publish_rate'),
        }],
    )
    
    return LaunchDescription([
        max_linear_arg,
        max_angular_arg,
        deadzone_arg,
        publish_rate_arg,
        teleop_node,
    ])

