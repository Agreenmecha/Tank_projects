#!/usr/bin/env python3
"""
Launch Point-LIO with Unitree L2 (Front) for tank localization
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('tank_localization'),
            'config',
            'point_lio_l2.yaml'
        ]),
        description='Path to Point-LIO config file'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )
    
    # Point-LIO node
    point_lio_node = Node(
        package='point_lio',
        executable='point_lio_node',
        name='point_lio',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            ('cloud_registered', '/point_lio/cloud'),
            ('Odometry', '/point_lio/odom'),
            ('path', '/point_lio/path'),
        ]
    )
    
    # RViz (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('tank_localization'),
            'config',
            'point_lio.rviz'
        ])],
        condition=launch.conditions.IfCondition(LaunchConfiguration('rviz'))
    )
    
    return LaunchDescription([
        config_arg,
        rviz_arg,
        point_lio_node,
        # rviz_node,  # Uncomment when rviz config is created
    ])

