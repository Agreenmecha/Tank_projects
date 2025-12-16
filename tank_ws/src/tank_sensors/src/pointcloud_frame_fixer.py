#!/usr/bin/env python3
"""
Republish LiDAR point clouds with fixed URDF frames and rotated orientation.

Goal:
- Keep IMU topics untouched (for localization, filtering, etc.)
- For visualization and mapping that assumes a rigid mount, publish
  point clouds that appear to come from the URDF mount links.
- Rotate point cloud 115° counter-clockwise around Z-axis to match physical orientation

Subscriptions:
- /lidar_front/cloud
- /lidar_rear/cloud

Publications:
- /lidar_front/cloud_fixed  (frame_id = URDF frame, rotated)
- /lidar_rear/cloud_fixed   (frame_id = URDF frame, rotated)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import struct


class PointcloudFrameFixer(Node):
    def __init__(self) -> None:
        super().__init__('pointcloud_frame_fixer')

        # Parameters for flexibility if we want to tweak later
        self.declare_parameter('front_input_topic', '/lidar_front/cloud')
        self.declare_parameter('rear_input_topic', '/lidar_rear/cloud')
        self.declare_parameter('front_output_topic', '/lidar_front/cloud_fixed')
        self.declare_parameter('rear_output_topic', '/lidar_rear/cloud_fixed')
        self.declare_parameter('front_frame', 'l_FL2')
        self.declare_parameter('rear_frame', 'l_BL2')
        self.declare_parameter('rotation_degrees', 115.0)  # Counter-clockwise around Z

        front_in = self.get_parameter('front_input_topic').get_parameter_value().string_value
        rear_in = self.get_parameter('rear_input_topic').get_parameter_value().string_value
        front_out = self.get_parameter('front_output_topic').get_parameter_value().string_value
        rear_out = self.get_parameter('rear_output_topic').get_parameter_value().string_value

        self.front_frame = self.get_parameter('front_frame').get_parameter_value().string_value
        self.rear_frame = self.get_parameter('rear_frame').get_parameter_value().string_value
        
        # Rotation angle (counter-clockwise around Z)
        rotation_deg = self.get_parameter('rotation_degrees').get_parameter_value().double_value
        self.rotation_rad = np.radians(rotation_deg)
        self.cos_theta = np.cos(self.rotation_rad)
        self.sin_theta = np.sin(self.rotation_rad)

        # Publishers
        self.front_pub = self.create_publisher(PointCloud2, front_out, 10)
        self.rear_pub = self.create_publisher(PointCloud2, rear_out, 10)

        # Subscribers
        self.front_sub = self.create_subscription(
            PointCloud2,
            front_in,
            self.front_cb,
            10,
        )
        self.rear_sub = self.create_subscription(
            PointCloud2,
            rear_in,
            self.rear_cb,
            10,
        )

        self.get_logger().info(
            f'PointcloudFrameFixer running:\n'
            f'  front: {front_in} -> {front_out} (frame={self.front_frame}, rotation={rotation_deg}°)\n'
            f'  rear:  {rear_in} -> {rear_out} (frame={self.rear_frame}, rotation={rotation_deg}°)'
        )

    def _rotate_and_publish(self, msg: PointCloud2, frame: str, pub) -> None:
        """Rotate point cloud and change frame_id."""
        try:
            # Read points from PointCloud2
            points_list = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            
            if len(points_list) == 0:
                self.get_logger().warn('Received empty point cloud')
                return
            
            # Extract x, y, z values from point tuples
            x = np.array([p[0] for p in points_list], dtype=np.float32)
            y = np.array([p[1] for p in points_list], dtype=np.float32)
            z = np.array([p[2] for p in points_list], dtype=np.float32)
            
            # Rotate around Z-axis (counter-clockwise)
            # x' = x*cos(θ) - y*sin(θ)
            # y' = x*sin(θ) + y*cos(θ)
            # z' = z (unchanged)
            
            x_rot = x * self.cos_theta - y * self.sin_theta
            y_rot = x * self.sin_theta + y * self.cos_theta
            z_rot = z
            
            # Create rotated points list
            rotated_points = np.column_stack((x_rot, y_rot, z_rot))
            
            # Create new PointCloud2 message
            msg_out = PointCloud2()
            msg_out.header = msg.header
            msg_out.header.frame_id = frame
            
            # Pack points back into PointCloud2 format
            msg_out.height = 1
            msg_out.width = len(rotated_points)
            msg_out.fields = msg.fields[:3]  # x, y, z fields
            msg_out.is_bigendian = False
            msg_out.point_step = 12  # 3 floats * 4 bytes
            msg_out.row_step = msg_out.point_step * msg_out.width
            msg_out.is_dense = True
            
            # Pack data
            buffer = []
            for point in rotated_points:
                buffer.append(struct.pack('fff', point[0], point[1], point[2]))
            msg_out.data = b''.join(buffer)
            
            pub.publish(msg_out)
            
        except Exception as e:
            self.get_logger().error(f'Error transforming point cloud: {e}')

    def front_cb(self, msg: PointCloud2) -> None:
        self._rotate_and_publish(msg, self.front_frame, self.front_pub)

    def rear_cb(self, msg: PointCloud2) -> None:
        self._rotate_and_publish(msg, self.rear_frame, self.rear_pub)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointcloudFrameFixer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
