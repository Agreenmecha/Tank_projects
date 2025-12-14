#!/usr/bin/env python3
"""
Simple joint state publisher that publishes zero states for all continuous joints.
This allows robot_state_publisher to publish TF transforms.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class SimpleJointStatePublisher(Node):
    def __init__(self):
        super().__init__('simple_joint_state_publisher')
        
        # All joints from the URDF (14 continuous + 2 fixed)
        self.joint_names = [
            'j_l1', 'j_l2', 'j_l3', 'j_l4', 'j_l5', 'j_l6', 'j_l7',
            'j_r1', 'j_r2', 'j_r3', 'j_r4', 'j_r5', 'j_r6', 'j_r7',
            'j_FL2', 'j_BL2'  # Fixed joints, but still need to be in joint_states
        ]
        
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10 Hz
        
        self.get_logger().info('Publishing joint states for %d joints' % len(self.joint_names))
    
    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.name = self.joint_names
        msg.position = [0.0] * len(self.joint_names)  # All zeros for continuous joints
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)
        
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleJointStatePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

