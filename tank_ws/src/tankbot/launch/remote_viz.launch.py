#!/usr/bin/env python3
"""
Launch file for remote visualization of tankbot with live sensor data.
Designed to run on local machine, visualizing data from Jetson Orin Nano.

Usage:
  # On Jetson: Launch sensors
  ros2 launch tank_sensors hardware.launch.py
  
  # On local machine:
  ros2 launch tankbot remote_viz.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('tankbot')
    
    # URDF file path
    urdf_path = os.path.join(pkg_share, 'urdf', 'tankbot.urdf')
    
    # Read URDF file
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Robot state publisher (publishes TF from URDF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Simple joint state publisher (publishes zeros for all joints)
    joint_state_script = os.path.join(pkg_share, 'launch', 'joint_state_publisher.py')
    
    from launch.actions import ExecuteProcess
    joint_state_publisher = ExecuteProcess(
        cmd=['python3', joint_state_script],
        output='screen',
        name='joint_state_publisher'
    )
    
    # Static transform from world to base_footprint
    static_tf_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_footprint'],
        output='screen'
    )
    
    # RViz with robot model
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['--ros-args', '-p', 'use_sim_time:=false'],
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        static_tf_world,
        robot_state_publisher,
        joint_state_publisher,
        rviz,
    ])

