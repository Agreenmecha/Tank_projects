#!/usr/bin/env python3
"""
Launch file for e-CAM25_CUONX camera using nvarguscamerasrc
Requires e-con Systems drivers to be installed first!
See: tank_ws/src/external/ECAM25_CAMERA_SETUP.md
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    width_arg = DeclareLaunchArgument(
        'width',
        default_value='1280',
        description='Camera width'
    )
    
    height_arg = DeclareLaunchArgument(
        'height',
        default_value='720',
        description='Camera height'
    )
    
    framerate_arg = DeclareLaunchArgument(
        'framerate',
        default_value='20',
        description='Camera framerate'
    )
    
    # GStreamer pipeline for nvarguscamerasrc (NVIDIA Argus API)
    # This pipeline:
    # 1. Captures from CSI camera using nvarguscamerasrc
    # 2. Converts from NVMM (GPU memory) to CPU memory
    # 3. Converts format to BGR for OpenCV/Isaac ROS compatibility
    gst_pipeline = (
        'nvarguscamerasrc sensor-id=0 ! '
        'video/x-raw(memory:NVMM), '
        'width={width}, height={height}, '
        'format=NV12, framerate={framerate}/1 ! '
        'nvvidconv ! '
        'video/x-raw, format=BGRx ! '
        'videoconvert ! '
        'video/x-raw, format=BGR ! '
        'appsink'
    )
    
    # gscam node for GStreamer-based camera
    camera_node = Node(
        package='gscam',
        executable='gscam_node',
        name='camera',
        output='screen',
        parameters=[{
            'camera_name': 'ecam25_cuonx',
            'camera_info_url': '',  # TODO: Add camera calibration file
            'frame_id': 'camera_link',
            'gscam_config': gst_pipeline.format(
                width=LaunchConfiguration('width'),
                height=LaunchConfiguration('height'),
                framerate=LaunchConfiguration('framerate')
            ),
            'sync_sink': True,
            'use_gst_timestamps': False,
            'image_encoding': 'bgr8',
        }],
        remappings=[
            ('camera/image_raw', '/camera/image_raw'),
            ('camera/camera_info', '/camera/camera_info'),
        ]
    )
    
    return LaunchDescription([
        width_arg,
        height_arg,
        framerate_arg,
        camera_node,
    ])

