#!/usr/bin/env python3
"""
Launch file for ZED-F9P GNSS
Uses aussierobots/ublox_dgnss driver with UBX protocol
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get tank_sensors config path
    tank_sensors_dir = get_package_share_directory('tank_sensors')
    config_file = os.path.join(tank_sensors_dir, 'config', 'gnss_f9p.yaml')
    
    # Declare arguments
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/ttyACM1',
        description='GNSS serial device'
    )
    
    # GNSS node (aussierobots driver) - publishes UBX messages
    gnss_node = Node(
        package='ublox_dgnss_node',
        executable='ublox_dgnss_node',
        name='gnss_node',
        output='screen',
        parameters=[
            config_file,
            {'device': LaunchConfiguration('device')}
        ],
    )
    
    # NavSatFix converter (UBX → /fix topic)
    # Subscribes to: /ubx_nav_hp_pos_llh, /ubx_nav_cov, /ubx_nav_status
    # Publishes: /fix (sensor_msgs/NavSatFix)
    navsatfix_node = Node(
        package='ublox_nav_sat_fix_hp_node',
        executable='ublox_nav_sat_fix_hp',
        name='ublox_nav_sat_fix_hp',
        output='screen',
    )
    
    return LaunchDescription([
        device_arg,
        gnss_node,
        navsatfix_node,
    ])
