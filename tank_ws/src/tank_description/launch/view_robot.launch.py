#!/usr/bin/env python3
"""
Launch RViz to visualize tank URDF model.
Useful for testing URDF before deploying to hardware.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Include robot state publisher
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tank_description'),
                'launch',
                'robot_state_publisher.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )
    
    # Joint state publisher GUI (for testing movable joints if any)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # RViz
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('tank_description'),
        'rviz',
        'view_robot.rviz'
    ])
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])

