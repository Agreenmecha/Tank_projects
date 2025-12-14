#!/usr/bin/env python3
"""
Launch RViz to visualize tankbot URDF model.
Exported from SolidWorks URDF exporter.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
    
    # Robot state publisher
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
    
    joint_state_publisher = ExecuteProcess(
        cmd=['python3', joint_state_script],
        output='screen',
        name='joint_state_publisher'
    )
    
    # Static transform from world to base_footprint (RViz needs a root frame)
    static_tf_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_footprint'],
        output='screen'
    )
    
    # RViz (user can load config manually)
    # Note: Set Fixed Frame to "world" in RViz, and make sure RobotModel
    # Description Source is set to "Topic" with topic "/robot_description"
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

