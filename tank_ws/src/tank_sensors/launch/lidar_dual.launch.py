#!/usr/bin/env python3
"""
Launch file for Dual Unitree L2 LiDARs - SIMPLE VERSION
- Front L2: Publishes with frame_id l_FL2 (URDF link)
- Rear L2: Publishes with frame_id l_BL2 (URDF link)
- No frame fixer needed - driver uses URDF frames directly
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    front_lidar_ip_arg = DeclareLaunchArgument(
        'front_lidar_ip',
        default_value='192.168.123.124',
        description='IP address of front L2 LiDAR'
    )
    
    rear_lidar_ip_arg = DeclareLaunchArgument(
        'rear_lidar_ip',
        default_value='192.168.123.123',
        description='IP address of rear L2 LiDAR'
    )
    
    local_ip_arg = DeclareLaunchArgument(
        'local_ip',
        default_value='192.168.2.100',
        description='Local IP address of this computer'
    )
    
    # Front L2 node - publishes directly with URDF frame
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
            'lidar_port': 6101,
            'local_ip': LaunchConfiguration('local_ip'),
            'local_port': 6201,
            'cloud_frame': 'l_FL2',  # URDF link frame
            'imu_frame': 'l_FL2_imu',
            'cloud_topic': 'cloud',
            'imu_topic': 'imu',
        }],
        remappings=[
            ('unilidar/cloud', '/lidar_front/cloud'),
            ('unilidar/imu', '/lidar_front/imu'),
        ],
        namespace='lidar_front'
    )
    
    # Rear L2 node - publishes directly with URDF frame
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
            'lidar_port': 6101,
            'local_ip': LaunchConfiguration('local_ip'),
            'local_port': 6202,
            'cloud_frame': 'l_BL2',  # URDF link frame
            'imu_frame': 'l_BL2_imu',
            'cloud_topic': 'cloud',
            'imu_topic': 'imu',
        }],
        remappings=[
            ('unilidar/cloud', '/lidar_rear/cloud'),
            ('unilidar/imu', '/lidar_rear/imu'),
        ],
        namespace='lidar_rear'
    )
    
    return LaunchDescription([
        front_lidar_ip_arg,
        rear_lidar_ip_arg,
        local_ip_arg,
        front_l2_node,
        rear_l2_node,
    ])
