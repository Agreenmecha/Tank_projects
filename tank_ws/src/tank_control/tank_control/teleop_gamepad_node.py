#!/usr/bin/env python3
"""
ROS2 Gamepad Teleop Node
Publishes Twist messages to /cmd_vel for ODrive control
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import sys

try:
    import pygame
except ImportError:
    print("❌ pygame not installed!")
    print("Install with: pip3 install pygame")
    sys.exit(1)


class TeleopGamepadNode(Node):
    """ROS2 node for gamepad teleoperation."""
    
    # Controller mapping for generic gamepad
    AXIS_LEFT_Y = 1      # Left stick vertical (forward/backward)
    AXIS_RIGHT_X = 2     # Right stick horizontal (turning)
    BUTTON_SQUARE = 3    # Square button - quit
    BUTTON_X = 2         # X button - emergency stop
    
    def __init__(self):
        super().__init__('teleop_gamepad_node')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_linear_vel', 1.5),    # m/s
                ('max_angular_vel', 2.0),   # rad/s
                ('deadzone', 0.15),         # Joystick deadzone
                ('publish_rate', 50.0),     # Hz
            ]
        )
        
        # Get parameters
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.deadzone = self.get_parameter('deadzone').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.estop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        
        # State
        self.emergency_stop = False
        
        # Initialize pygame
        self.joystick = self.init_pygame()
        if self.joystick is None:
            self.get_logger().error('Failed to initialize gamepad')
            raise RuntimeError('No gamepad detected')
        
        # Create timer for publishing
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.print_info()
    
    def init_pygame(self):
        """Initialize pygame and find gamepad."""
        import os
        os.environ['SDL_JOYSTICK_DEVICE'] = '/dev/input/event*'
        
        pygame.init()
        pygame.joystick.init()
        
        import time
        time.sleep(0.5)
        pygame.event.pump()
        
        if pygame.joystick.get_count() == 0:
            self.get_logger().error('No gamepad detected!')
            return None
        
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        
        self.get_logger().info(f'Connected to: {joystick.get_name()}')
        self.get_logger().info(f'Axes: {joystick.get_numaxes()}, Buttons: {joystick.get_numbuttons()}')
        
        return joystick
    
    def apply_deadzone(self, value):
        """Apply deadzone to joystick input."""
        if abs(value) < self.deadzone:
            return 0.0
        # Scale the remaining range to 0-1
        sign = 1 if value > 0 else -1
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)
    
    def timer_callback(self):
        """Read gamepad and publish velocity commands."""
        # Process pygame events
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == self.BUTTON_X:
                    self.emergency_stop = True
                    self.get_logger().warn('🛑 EMERGENCY STOP')
                    # Publish e-stop
                    estop_msg = Bool()
                    estop_msg.data = True
                    self.estop_pub.publish(estop_msg)
                elif event.button == self.BUTTON_SQUARE:
                    self.get_logger().info('Quit button pressed')
                    rclpy.shutdown()
            elif event.type == pygame.JOYBUTTONUP:
                if event.button == self.BUTTON_X:
                    self.emergency_stop = False
                    self.get_logger().info('▶️  Resume')
                    # Clear e-stop
                    estop_msg = Bool()
                    estop_msg.data = False
                    self.estop_pub.publish(estop_msg)
        
        # Create Twist message
        twist = Twist()
        
        if not self.emergency_stop:
            # Left stick Y - forward/backward (inverted)
            forward = -self.apply_deadzone(self.joystick.get_axis(self.AXIS_LEFT_Y))
            
            # Right stick X - turning
            turn = self.apply_deadzone(self.joystick.get_axis(self.AXIS_RIGHT_X))
            
            # Apply velocity limits
            twist.linear.x = forward * self.max_linear_vel
            twist.angular.z = turn * self.max_angular_vel
        else:
            # E-stop - zero velocity
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        # Publish
        self.cmd_vel_pub.publish(twist)
    
    def print_info(self):
        """Print control information."""
        self.get_logger().info('='*50)
        self.get_logger().info('ROS2 Gamepad Teleop Node')
        self.get_logger().info('='*50)
        self.get_logger().info('Controls:')
        self.get_logger().info('  Left Stick Y    : Forward/Backward')
        self.get_logger().info('  Right Stick X   : Turn Left/Right')
        self.get_logger().info('  X Button        : Emergency Stop')
        self.get_logger().info('  Square Button   : Quit')
        self.get_logger().info(f'Max Linear Vel:  {self.max_linear_vel} m/s')
        self.get_logger().info(f'Max Angular Vel: {self.max_angular_vel} rad/s')
        self.get_logger().info(f'Deadzone: {self.deadzone}')
        self.get_logger().info('='*50)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TeleopGamepadNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        # Cleanup
        if rclpy.ok():
            # Send zero velocity
            if 'node' in locals():
                twist = Twist()
                node.cmd_vel_pub.publish(twist)
                node.get_logger().info('Stopped robot')
            rclpy.shutdown()
        pygame.quit()


if __name__ == '__main__':
    main()

