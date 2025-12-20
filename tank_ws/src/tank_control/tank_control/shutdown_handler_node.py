#!/usr/bin/env python3

"""
Shutdown Handler Node
Listens for shutdown commands and terminates all ROS nodes cleanly.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import sys
import os
import signal
import subprocess


class ShutdownHandlerNode(Node):
    """Node that handles emergency shutdown commands"""
    
    def __init__(self):
        super().__init__('shutdown_handler')
        
        # Subscribe to shutdown topic
        self.shutdown_sub = self.create_subscription(
            Bool,
            '/system/shutdown',
            self.shutdown_callback,
            10
        )
        
        self.get_logger().info('Shutdown Handler Node initialized')
        self.get_logger().warn('Listening for shutdown commands on /system/shutdown')
    
    def shutdown_callback(self, msg: Bool):
        """Handle shutdown command"""
        if msg.data:
            self.get_logger().fatal('⚠️ SHUTDOWN COMMAND RECEIVED!')
            self.get_logger().fatal('Terminating all ROS nodes...')
            
            # Give time for the log message to be published
            rclpy.spin_once(self, timeout_sec=0.5)
            
            # Kill all ROS2 processes
            try:
                # Find parent launch process and kill entire process group
                self.get_logger().info('Killing all ROS2 nodes...')
                subprocess.run(['pkill', '-9', '-f', 'ros2'], check=False)
                subprocess.run(['pkill', '-9', '-f', 'launch'], check=False)
            except Exception as e:
                self.get_logger().error(f'Error during shutdown: {e}')
            
            # Force exit this node
            sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = ShutdownHandlerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

