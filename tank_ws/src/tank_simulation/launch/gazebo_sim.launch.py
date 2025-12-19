import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    # Get package directories
    pkg_tank_sim = get_package_share_directory('tank_simulation')
    pkg_master_bot = get_package_share_directory('master_bot')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # World file
    world_file = os.path.join(pkg_tank_sim, 'worlds', 'outdoor_test.world')
    
    # URDF file
    urdf_file = os.path.join(pkg_master_bot, 'urdf', 'master_bot.urdf')
    
    # Read URDF
    with open(urdf_file, 'r') as file:
        robot_desc = file.read()
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )
    
    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )
    
    # Spawn robot at ground level (base_footprint is at ground)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'tank',
            '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',  # base_footprint starts at ground
        ],
        output='screen'
    )
    
    # Clock bridge
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    
    # LiDAR bridges
    lidar_front_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/lidar_front/pointcloud@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'],
        remappings=[('/lidar_front/pointcloud', '/lidar_front/cloud')],
        output='screen'
    )
    
    lidar_rear_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/lidar_rear/pointcloud@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'],
        remappings=[('/lidar_rear/pointcloud', '/lidar_rear/cloud')],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_robot,
        clock_bridge,
        lidar_front_bridge,
        lidar_rear_bridge,
    ])

