#!/usr/bin/env python3
"""
Simple launch file to view raw LiDAR point clouds without robot model.
Sets Fixed Frame to lidar_front so no transforms are needed.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # RViz with Fixed Frame set to lidar_front (no transforms needed)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['--ros-args', '-p', 'use_sim_time:=false'],
        parameters=[{
            'use_sim_time': False
        }]
    )
    
    return LaunchDescription([
        rviz,
    ])

