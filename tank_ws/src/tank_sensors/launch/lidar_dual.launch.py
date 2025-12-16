#!/usr/bin/env python3
"""
Launch file for Dual Unitree L2 LiDARs + Frame Fixer
- Driver publishes with its own frames (lidar_front_raw, lidar_rear_raw)
- Frame fixer republishes with URDF frames (l_FL2, l_BL2)
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
    
    # Front L2 node
    front_l2_node = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='lidar_front',
        output='screen',
        parameters=[{
            'initialize_type': 2,
            'work_mode': 0,
            'use_system_timestamp': True,
            'range_min': 0.05,
            'range_max': 30.0,
            'cloud_scan_num': 18,
            'lidar_ip': LaunchConfiguration('front_lidar_ip'),
            'lidar_port': 6101,
            'local_ip': LaunchConfiguration('local_ip'),
            'local_port': 6201,
            # Frame IDs (driver will use these)
            'cloud_frame': 'unilidar_lidar',  # Driver default frame
            'imu_frame': 'unilidar_imu',      # Driver default frame
            'cloud_topic': 'cloud',
            'imu_topic': 'imu',
        }],
        remappings=[
            ('unilidar/cloud', '/lidar_front/cloud'),
            ('unilidar/imu', '/lidar_front/imu'),
        ],
        namespace='lidar_front'
    )
    
    # Rear L2 node
    rear_l2_node = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='lidar_rear',
        output='screen',
        parameters=[{
            'initialize_type': 2,
            'work_mode': 0,
            'use_system_timestamp': True,
            'range_min': 0.05,
            'range_max': 30.0,
            'cloud_scan_num': 18,
            'lidar_ip': LaunchConfiguration('rear_lidar_ip'),
            'lidar_port': 6101,
            'local_ip': LaunchConfiguration('local_ip'),
            'local_port': 6202,
            # Frame IDs (driver will use these)
            'cloud_frame': 'unilidar_lidar',  # Driver default frame
            'imu_frame': 'unilidar_imu',      # Driver default frame
            'cloud_topic': 'cloud',
            'imu_topic': 'imu',
        }],
        remappings=[
            ('unilidar/cloud', '/lidar_rear/cloud'),
            ('unilidar/imu', '/lidar_rear/imu'),
        ],
        namespace='lidar_rear'
    )
    
    # Frame fixer: republish clouds with URDF frame_ids and rotate orientation
    # Subscribes to driver output, republishes with URDF frames for RViz
    # URDF link names: forward position = l_BL2, rear position = l_FL2
    # Driver topics: /lidar_front = forward sensor, /lidar_rear = rear sensor
    # Rotates point cloud 115° counter-clockwise around Z to match physical orientation
    frame_fixer = Node(
        package='tank_sensors',
        executable='pointcloud_frame_fixer.py',
        name='pointcloud_frame_fixer',
        output='screen',
        parameters=[{
            'front_input_topic': '/lidar_front/cloud',
            'rear_input_topic': '/lidar_rear/cloud',
            'front_output_topic': '/lidar_front/cloud_fixed',
            'rear_output_topic': '/lidar_rear/cloud_fixed',
            'front_frame': 'l_FL2',  # Front sensor data → l_FL2 URDF frame
            'rear_frame': 'l_BL2',   # Rear sensor data → l_BL2 URDF frame
            'rotation_degrees': -115.0,  # Clockwise around Z-axis (negative = CW)
        }]
    )
    
    return LaunchDescription([
        front_lidar_ip_arg,
        rear_lidar_ip_arg,
        local_ip_arg,
        front_l2_node,
        rear_l2_node,
        frame_fixer,
    ])
