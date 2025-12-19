#!/usr/bin/env python3
"""
Tank Navigation Launch File
Launches Nav2 stack for autonomous navigation
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launch Nav2 stack with tank-specific configuration
    
    Prerequisites:
    - Point-LIO running (provides /Odometry and map→base_link TF)
    - LiDAR drivers running (provides point clouds)
    - Robot state publisher (provides robot TF tree)
    """
    
    # Get package directories
    tank_nav_dir = get_package_share_directory('tank_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Paths to config files
    nav2_params_file = os.path.join(tank_nav_dir, 'config', 'nav2', 'nav2_params.yaml')
    
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_file,
        description='Full path to Nav2 params file'
    )
    
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start Nav2 lifecycle nodes'
    )
    
    use_composition_arg = DeclareLaunchArgument(
        'use_composition',
        default_value='false',
        description='Use composed bringup for Nav2'
    )
    
    use_respawn_arg = DeclareLaunchArgument(
        'use_respawn',
        default_value='false',
        description='Whether to respawn if a node crashes'
    )
    
    # Nav2 bringup
    # This launches all Nav2 nodes with our custom params
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': LaunchConfiguration('autostart'),
            'use_composition': LaunchConfiguration('use_composition'),
            'use_respawn': LaunchConfiguration('use_respawn'),
        }.items()
    )
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        params_file_arg,
        autostart_arg,
        use_composition_arg,
        use_respawn_arg,
        
        # Nav2 stack
        nav2_bringup,
    ])

