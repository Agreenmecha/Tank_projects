#!/usr/bin/env python3
"""
Launch robot_state_publisher for tank URDF.
Publishes TF transforms for all sensors based on URDF.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Get URDF file path
    urdf_file = PathJoinSubstitution([
        FindPackageShare('tank_description'),
        'urdf',
        'tank.urdf.xacro'
    ])
    
    # Process xacro file to get robot description
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_frequency': 50.0,  # Hz
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher,
    ])

