#!/usr/bin/env python3
"""
Complete motor control stack launch file with gamepad override
Launches:
- Twist Mux (prioritizes gamepad > Nav2 > safety)
- ODrive interface (motor controller)
- Safety monitor (emergency stop, limits)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
    
    twist_mux_config_arg = DeclareLaunchArgument(
        'twist_mux_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('tank_control'),
            'config',
            'twist_mux.yaml'
        ]),
        description='Path to twist_mux configuration file'
    )
    
    enable_safety_arg = DeclareLaunchArgument(
        'enable_safety',
        default_value='true',
        description='Enable safety monitor node'
    )
    
    enable_mux_arg = DeclareLaunchArgument(
        'enable_mux',
        default_value='true',
        description='Enable twist_mux for gamepad override'
    )
    
    # Twist Mux Node (prioritizes gamepad over Nav2)
    # Allows remote gamepad to override autonomous navigation
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[LaunchConfiguration('twist_mux_config')],
        remappings=[
            # Output to ODrive
            ('cmd_vel_out', '/cmd_vel'),
        ],
        condition=IfCondition(LaunchConfiguration('enable_mux'))
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
        condition=IfCondition(LaunchConfiguration('enable_safety')),
    )
    
    return LaunchDescription([
        odrive_config_arg,
        safety_config_arg,
        twist_mux_config_arg,
        enable_safety_arg,
        enable_mux_arg,
        twist_mux_node,
        odrive_node,
        safety_node,
    ])
