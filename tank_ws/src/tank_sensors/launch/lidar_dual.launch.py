#!/usr/bin/env python3
"""
Launch file for Dual Unitree L2 LiDARs
- Front L2: Used for Point-LIO localization
- Rear L2: Used for rear perception and costmap
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    front_ip_arg = DeclareLaunchArgument(
        'front_lidar_ip',
        default_value='192.168.123.123',
        description='IP address of front L2 LiDAR'
    )
    
    rear_ip_arg = DeclareLaunchArgument(
        'rear_lidar_ip',
        default_value='192.168.123.124',
        description='IP address of rear L2 LiDAR (change with Unitree host software)'
    )
    
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='18888',
        description='UDP port for L2 LiDAR'
    )
    
    # Front L2 node (for Point-LIO)
    front_l2_node = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='lidar_front',
        output='screen',
        parameters=[{
            'lidar_ip': LaunchConfiguration('front_lidar_ip'),
            'port': LaunchConfiguration('port'),
            'frame_id': 'lidar_front',
            'cloud_topic': 'cloud',
            'imu_topic': 'imu',
        }],
        remappings=[
            ('unilidar/cloud', '/lidar_front/pointcloud'),
            ('unilidar/imu', '/lidar_front/imu'),
        ],
        namespace='lidar_front'
    )
    
    # Rear L2 node (for rear perception)
    rear_l2_node = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='lidar_rear',
        output='screen',
        parameters=[{
            'lidar_ip': LaunchConfiguration('rear_lidar_ip'),
            'port': LaunchConfiguration('port'),
            'frame_id': 'lidar_rear',
            'cloud_topic': 'cloud',
            'imu_topic': 'imu',
        }],
        remappings=[
            ('unilidar/cloud', '/lidar_rear/pointcloud'),
            ('unilidar/imu', '/lidar_rear/imu'),  # Not used, but published
        ],
        namespace='lidar_rear'
    )
    
    return LaunchDescription([
        front_ip_arg,
        rear_ip_arg,
        port_arg,
        front_l2_node,
        rear_l2_node,
    ])

