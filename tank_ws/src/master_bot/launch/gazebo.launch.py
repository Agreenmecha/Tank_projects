import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directory
    pkg_share = FindPackageShare('master_bot').find('master_bot')
    
    # Set Gazebo resource path so it can find package:// URIs
    gz_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=os.path.join(pkg_share, '..')  # Parent directory containing package
    )
    
    # Path to URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'master_bot.urdf')
    
    # Read the URDF file
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()
    
    # Gazebo Fortress (gz sim) launch file
    gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-r empty.sdf'
        }.items()
    )
    
    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )
    
    # Spawn the robot in Gazebo Fortress
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'master_bot',
            '-file', urdf_file,
            '-x', '0',
            '-y', '0',
            '-z', '0.5'
        ],
        output='screen'
    )
    
    # Bridge to convert Gazebo topics to ROS 2 topics
    bridge_front_lidar = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/lidar_front/pointcloud@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'
        ],
        output='screen'
    )
    
    bridge_rear_lidar = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/lidar_rear/pointcloud@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        gz_resource_path,
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
        bridge_front_lidar,
        bridge_rear_lidar
    ])

