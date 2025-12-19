#!/usr/bin/env python3
"""
Tank Localization: Point-LIO Launcher
Launches Point-LIO with tank-specific configuration for Unitree L2 front LiDAR
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch Point-LIO for tank localization
    
    This wraps the Point-LIO package and launches it with tank-specific settings.
    Point-LIO uses the front L2 LiDAR for odometry and mapping.
    """
    
    # Declare arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )
    
    # Include Point-LIO launch file
    # Uses the config we set up in point_lio/config/unilidar_l2.yaml
    point_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('point_lio'),
                'launch',
                'mapping_unilidar_l2.launch.py'
            ])
        ]),
        launch_arguments={
            'rviz': LaunchConfiguration('rviz'),
        }.items()
    )
    
    return LaunchDescription([
        rviz_arg,
        point_lio_launch,
    ])
