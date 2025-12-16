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
    front_lidar_ip_arg = DeclareLaunchArgument(
        'front_lidar_ip',
        default_value='192.168.123.123',
        description='IP address of front L2 LiDAR'
    )
    
    rear_lidar_ip_arg = DeclareLaunchArgument(
        'rear_lidar_ip',
        default_value='192.168.123.124',
        description='IP address of rear L2 LiDAR (change with Unitree host software)'
    )
    
    front_lidar_port_arg = DeclareLaunchArgument(
        'front_lidar_port',
        default_value='6101',
        description='UDP port on front L2 LiDAR (default: 6101)'
    )
    
    rear_lidar_port_arg = DeclareLaunchArgument(
        'rear_lidar_port',
        default_value='6101',
        description='UDP port on rear L2 LiDAR (default: 6101)'
    )
    
    front_local_port_arg = DeclareLaunchArgument(
        'front_local_port',
        default_value='6201',
        description='Local UDP port to receive data from front L2 (default: 6201)'
    )
    
    rear_local_port_arg = DeclareLaunchArgument(
        'rear_local_port',
        default_value='6202',
        description='Local UDP port to receive data from rear L2 (default: 6202, must be different from front)'
    )
    
    local_ip_arg = DeclareLaunchArgument(
        'local_ip',
        default_value='192.168.2.100',
        description='Local IP address of this computer'
    )
    
    # Front L2 node (for Point-LIO)
    front_l2_node = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='lidar_front',
        output='screen',
        parameters=[{
            'initialize_type': 2,  # UDP mode
            'work_mode': 0,  # Normal mode
            'use_system_timestamp': True,
            'range_min': 0.05,
            'range_max': 30.0,
            'cloud_scan_num': 18,
            'lidar_ip': LaunchConfiguration('front_lidar_ip'),
            'lidar_port': LaunchConfiguration('front_lidar_port'),
            'local_ip': LaunchConfiguration('local_ip'),
            'local_port': LaunchConfiguration('front_local_port'),
            'cloud_frame': 'lidar_front',  # Use driver's own frame
            'imu_frame': 'lidar_front_imu',
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
            'initialize_type': 2,  # UDP mode
            'work_mode': 0,  # Normal mode
            'use_system_timestamp': True,
            'range_min': 0.05,
            'range_max': 30.0,
            'cloud_scan_num': 18,
            'lidar_ip': LaunchConfiguration('rear_lidar_ip'),
            'lidar_port': LaunchConfiguration('rear_lidar_port'),
            'local_ip': LaunchConfiguration('local_ip'),
            'local_port': LaunchConfiguration('rear_local_port'),
            'cloud_frame': 'lidar_rear',  # Use driver's own frame
            'imu_frame': 'lidar_rear_imu',
            'cloud_topic': 'cloud',
            'imu_topic': 'imu',
        }],
        remappings=[
            ('unilidar/cloud', '/lidar_rear/pointcloud'),
            ('unilidar/imu', '/lidar_rear/imu'),
        ],
        namespace='lidar_rear'
    )
    
    return LaunchDescription([
        front_lidar_ip_arg,
        rear_lidar_ip_arg,
        front_lidar_port_arg,
        rear_lidar_port_arg,
        front_local_port_arg,
        rear_local_port_arg,
        local_ip_arg,
        front_l2_node,
        rear_l2_node,
    ])

