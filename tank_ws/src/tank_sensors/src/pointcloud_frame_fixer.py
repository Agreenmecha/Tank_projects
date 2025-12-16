#!/usr/bin/env python3
"""
Republish LiDAR point clouds with fixed URDF frames.

Goal:
- Keep IMU topics untouched (for localization, filtering, etc.)
- For visualization and mapping that assumes a rigid mount, publish
  point clouds that appear to come from the URDF mount links.

Subscriptions:
- /lidar_front/cloud
- /lidar_rear/cloud

Publications:
- /lidar_front/cloud_fixed  (frame_id = "l_FL2")
- /lidar_rear/cloud_fixed   (frame_id = "l_BL2")
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


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

        front_in = self.get_parameter('front_input_topic').get_parameter_value().string_value
        rear_in = self.get_parameter('rear_input_topic').get_parameter_value().string_value
        front_out = self.get_parameter('front_output_topic').get_parameter_value().string_value
        rear_out = self.get_parameter('rear_output_topic').get_parameter_value().string_value

        self.front_frame = self.get_parameter('front_frame').get_parameter_value().string_value
        self.rear_frame = self.get_parameter('rear_frame').get_parameter_value().string_value

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
            f'  front: {front_in} -> {front_out} (frame={self.front_frame})\n'
            f'  rear:  {rear_in} -> {rear_out} (frame={self.rear_frame})'
        )

    def _fix_and_publish(self, msg: PointCloud2, frame: str, pub) -> None:
        # Shallow copy is enough; we only tweak header.frame_id
        msg_out = PointCloud2()
        msg_out = msg  # reuse allocation; header will be overwritten
        msg_out.header.frame_id = frame
        pub.publish(msg_out)

    def front_cb(self, msg: PointCloud2) -> None:
        self._fix_and_publish(msg, self.front_frame, self.front_pub)

    def rear_cb(self, msg: PointCloud2) -> None:
        self._fix_and_publish(msg, self.rear_frame, self.rear_pub)


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


