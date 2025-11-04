"""
Dual Unitree L2 LiDAR Launch File
Configured for:
- Front LiDAR: 192.168.2.62 → port 6201
- Rear LiDAR: 192.168.2.63 → port 6202
- Jetson: 192.168.2.100
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Front LiDAR Node
    front_lidar_node = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='lidar_front',
        output='screen',
        parameters=[
            {'initialize_type': 2},  # UDP mode
            {'work_mode': 0},        # Normal mode
            {'use_system_timestamp': True},
            {'range_min': 0.05},     # 5cm min range
            {'range_max': 30.0},     # 30m max range
            {'cloud_scan_num': 18},  # Point cloud accumulation
            
            # Network settings - Front LiDAR
            {'lidar_port': 6101},
            {'lidar_ip': '192.168.2.62'},
            {'local_port': 6201},
            {'local_ip': '192.168.2.100'},
            
            # Frame and topic names
            {'cloud_frame': 'lidar_front'},
            {'cloud_topic': '/lidar_front/pointcloud'},
            {'imu_frame': 'lidar_front_imu'},
            {'imu_topic': '/lidar_front/imu'},
        ]
    )

    # Rear LiDAR Node
    rear_lidar_node = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='lidar_rear',
        output='screen',
        parameters=[
            {'initialize_type': 2},  # UDP mode
            {'work_mode': 0},        # Normal mode
            {'use_system_timestamp': True},
            {'range_min': 0.05},     # 5cm min range
            {'range_max': 30.0},     # 30m max range
            {'cloud_scan_num': 18},  # Point cloud accumulation
            
            # Network settings - Rear LiDAR  
            {'lidar_port': 6101},
            {'lidar_ip': '192.168.2.63'},
            {'local_port': 6202},
            {'local_ip': '192.168.2.100'},
            
            # Frame and topic names
            {'cloud_frame': 'lidar_rear'},
            {'cloud_topic': '/lidar_rear/pointcloud'},
            {'imu_frame': 'lidar_rear_imu'},
            {'imu_topic': '/lidar_rear/imu'},
        ]
    )

    return LaunchDescription([
        front_lidar_node,
        rear_lidar_node,
    ])

