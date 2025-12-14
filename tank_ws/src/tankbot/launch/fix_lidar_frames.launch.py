#!/usr/bin/env python3
"""
Add static transforms for LiDAR frames if driver uses 'unilidar' frame_id.
This is a workaround if the LiDAR driver doesn't respect the frame_id parameter.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Static transform from unilidar to lidar_front (identity - same frame)
    # If driver publishes with frame_id='unilidar', this makes it available as lidar_front
    unilidar_to_lidar_front = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='unilidar_to_lidar_front',
        arguments=['0', '0', '0', '0', '0', '0', 'unilidar', 'lidar_front'],
        output='screen'
    )
    
    # Static transform for rear (if using same frame name)
    # Note: If rear also uses 'unilidar', you may need a different approach
    # This assumes each LiDAR uses its own namespace
    
    return LaunchDescription([
        unilidar_to_lidar_front,
    ])

