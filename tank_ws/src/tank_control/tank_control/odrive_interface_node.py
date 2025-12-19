#!/usr/bin/env python3
"""
ODrive USB Interface Node for Tank Control
Supports: ODrive v3.6 with Firmware 0.5.6
Interface: USB (native protocol via odrivetool)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Trigger
import odrive
from odrive.enums import *
import time
import math
import threading


class ODriveInterfaceNode(Node):
    """
    ROS2 interface for ODrive motor controller via USB.
    
    Subscribes:
        /cmd_vel (geometry_msgs/Twist) - Velocity commands
        /emergency_stop (std_msgs/Bool) - Emergency stop trigger
    
    Publishes:
        /odrive/encoder_odom (nav_msgs/Odometry) - Wheel encoder odometry
        /odrive/motor_status (sensor_msgs/JointState) - Motor velocities, currents, temps
        /odrive/errors (std_msgs/Bool) - Error status
    
    Services:
        /odrive/clear_errors (std_srvs/Trigger) - Clear ODrive errors
        /odrive/calibrate (std_srvs/Trigger) - Run motor calibration
    """
    
    def __init__(self):
        super().__init__('odrive_interface_node')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('odrive_serial', ''),  # Empty = auto-find first ODrive
                ('axis_left', 0),       # ODrive axis for left motor
                ('axis_right', 1),      # ODrive axis for right motor
                ('wheel_radius', 0.058),  # meters (58mm actual wheel radius)
                ('track_width', 0.60),   # meters (600mm track width)
                ('encoder_cpr', 2048),   # Counts per revolution at motor (actual from ODrive)
                ('gear_ratio', 12.0),    # 12:1 gearbox reduction (motor:wheel)
                ('max_vel', 1.5),        # m/s
                ('max_angular_vel', 2.0), # rad/s
                ('watchdog_timeout', 1.0), # seconds
                ('control_rate', 50.0),    # Hz
                ('publish_rate', 20.0),    # Hz
                ('current_limit', 12.0),   # Amps (actual from ODrive)
                ('velocity_limit', 100.0), # turns/s at motor (actual from ODrive)
                ('vel_ramp_rate', 50.0),   # turns/s^2 - acceleration ramp rate (actual from ODrive)
                ('enable_watchdog', True),
            ]
        )
        
        # Get parameters
        self.odrive_serial = self.get_parameter('odrive_serial').value
        self.axis_left = self.get_parameter('axis_left').value
        self.axis_right = self.get_parameter('axis_right').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.track_width = self.get_parameter('track_width').value
        self.encoder_cpr = self.get_parameter('encoder_cpr').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        self.max_vel = self.get_parameter('max_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.watchdog_timeout = self.get_parameter('watchdog_timeout').value
        self.control_rate = self.get_parameter('control_rate').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.current_limit = self.get_parameter('current_limit').value
        self.velocity_limit = self.get_parameter('velocity_limit').value
        self.vel_ramp_rate = self.get_parameter('vel_ramp_rate').value
        self.enable_watchdog = self.get_parameter('enable_watchdog').value
        
        # State variables
        self.odrv = None
        self.connected = False
        self.e_stop = False
        self.last_cmd_time = time.time()
        self.target_vel_left = 0.0
        self.target_vel_right = 0.0
        
        # Odometry state
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.last_odom_time = time.time()
        self.last_left_pos = 0.0
        self.last_right_pos = 0.0
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        self.estop_sub = self.create_subscription(
            Bool, '/emergency_stop', self.estop_callback, 10
        )
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odrive/encoder_odom', 10)
        self.status_pub = self.create_publisher(JointState, '/odrive/motor_status', 10)
        self.error_pub = self.create_publisher(Bool, '/odrive/errors', 10)
        
        # Services
        self.clear_errors_srv = self.create_service(
            Trigger, '/odrive/clear_errors', self.clear_errors_callback
        )
        self.calibrate_srv = self.create_service(
            Trigger, '/odrive/calibrate', self.calibrate_callback
        )
        
        # Timers
        self.control_timer = self.create_timer(
            1.0 / self.control_rate, self.control_loop
        )
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate, self.publish_status
        )
        
        # Connect to ODrive
        self.connect_odrive()
        
        self.get_logger().info('ODrive Interface Node initialized')
    
    def connect_odrive(self):
        """Connect to ODrive via USB."""
        self.get_logger().info('Searching for ODrive...')
        try:
            if self.odrive_serial:
                self.get_logger().info(f'Looking for ODrive with serial: {self.odrive_serial}')
                self.odrv = odrive.find_any(serial_number=self.odrive_serial, timeout=10)
            else:
                self.get_logger().info('Looking for any ODrive...')
                self.odrv = odrive.find_any(timeout=10)
            
            if self.odrv is None:
                self.get_logger().error('ODrive not found!')
                return
            
            self.get_logger().info(f'Connected to ODrive: {self.odrv.serial_number}')
            self.get_logger().info(f'Firmware version: {self.odrv.fw_version_major}.{self.odrv.fw_version_minor}.{self.odrv.fw_version_revision}')
            
            # Use existing ODrive configuration (do not modify)
            self.get_logger().info('Using existing ODrive configuration')
            self.log_odrive_config()
            self.connected = True
            
        except Exception as e:
            self.get_logger().error(f'Failed to connect to ODrive: {e}')
            self.connected = False
    
    def log_odrive_config(self):
        """Log existing ODrive configuration (read-only)."""
        try:
            # Get axis objects
            left_axis = self.odrv.axis0 if self.axis_left == 0 else self.odrv.axis1
            right_axis = self.odrv.axis0 if self.axis_right == 0 else self.odrv.axis1
            
            self.get_logger().info('=== ODrive Configuration (Read-Only) ===')
            self.get_logger().info(f'Left Axis (axis{self.axis_left}):')
            self.get_logger().info(f'  Current limit: {left_axis.motor.config.current_lim}A')
            self.get_logger().info(f'  Velocity limit: {left_axis.controller.config.vel_limit} turns/s')
            self.get_logger().info(f'  Vel ramp rate: {left_axis.controller.config.vel_ramp_rate} turns/s²')
            self.get_logger().info(f'  Control mode: {left_axis.controller.config.control_mode}')
            self.get_logger().info(f'  Input mode: {left_axis.controller.config.input_mode}')
            self.get_logger().info(f'  Watchdog enabled: {left_axis.config.enable_watchdog}')
            self.get_logger().info(f'  Watchdog timeout: {left_axis.config.watchdog_timeout}s')
            
            self.get_logger().info(f'Right Axis (axis{self.axis_right}):')
            self.get_logger().info(f'  Current limit: {right_axis.motor.config.current_lim}A')
            self.get_logger().info(f'  Velocity limit: {right_axis.controller.config.vel_limit} turns/s')
            self.get_logger().info(f'  Vel ramp rate: {right_axis.controller.config.vel_ramp_rate} turns/s²')
            self.get_logger().info(f'  Control mode: {right_axis.controller.config.control_mode}')
            self.get_logger().info(f'  Input mode: {right_axis.controller.config.input_mode}')
            self.get_logger().info(f'  Watchdog enabled: {right_axis.config.enable_watchdog}')
            self.get_logger().info(f'  Watchdog timeout: {right_axis.config.watchdog_timeout}s')
            
            # Check for errors
            self.check_errors()
            
        except Exception as e:
            self.get_logger().error(f'Failed to read ODrive config: {e}')
    
    def check_errors(self):
        """Check for ODrive errors and publish status."""
        try:
            left_axis = self.odrv.axis0 if self.axis_left == 0 else self.odrv.axis1
            right_axis = self.odrv.axis0 if self.axis_right == 0 else self.odrv.axis1
            
            has_error = False
            
            # Check axis errors
            if left_axis.error != 0:
                self.get_logger().error(f'Left axis error: {hex(left_axis.error)}')
                has_error = True
            if right_axis.error != 0:
                self.get_logger().error(f'Right axis error: {hex(right_axis.error)}')
                has_error = True
            
            # Check motor errors
            if left_axis.motor.error != 0:
                self.get_logger().error(f'Left motor error: {hex(left_axis.motor.error)}')
                has_error = True
            if right_axis.motor.error != 0:
                self.get_logger().error(f'Right motor error: {hex(right_axis.motor.error)}')
                has_error = True
            
            # Check encoder errors
            if left_axis.encoder.error != 0:
                self.get_logger().error(f'Left encoder error: {hex(left_axis.encoder.error)}')
                has_error = True
            if right_axis.encoder.error != 0:
                self.get_logger().error(f'Right encoder error: {hex(right_axis.encoder.error)}')
                has_error = True
            
            # Publish error status
            error_msg = Bool()
            error_msg.data = has_error
            self.error_pub.publish(error_msg)
            
            return has_error
            
        except Exception as e:
            self.get_logger().error(f'Error checking ODrive errors: {e}')
            return True
    
    def cmd_vel_callback(self, msg):
        """Handle incoming velocity commands."""
        if not self.connected or self.e_stop:
            return
        
        # Convert Twist to differential drive velocities
        linear_vel = msg.linear.x  # m/s
        angular_vel = msg.angular.z  # rad/s
        
        # Clamp velocities
        linear_vel = max(-self.max_vel, min(self.max_vel, linear_vel))
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))
        
        # Differential drive kinematics
        # v_left = v - (w * track_width / 2)
        # v_right = v + (w * track_width / 2)
        vel_left = linear_vel - (angular_vel * self.track_width / 2.0)
        vel_right = linear_vel + (angular_vel * self.track_width / 2.0)
        
        # Convert to motor velocity (turns/s)
        # Wheel: turns/s = (m/s) / (2 * pi * wheel_radius)
        # Motor: wheel_turns/s * gear_ratio (motor spins faster than wheel)
        wheel_vel_left = vel_left / (2.0 * math.pi * self.wheel_radius)
        wheel_vel_right = vel_right / (2.0 * math.pi * self.wheel_radius)
        self.target_vel_left = wheel_vel_left * self.gear_ratio
        self.target_vel_right = wheel_vel_right * self.gear_ratio
        
        self.last_cmd_time = time.time()
    
    def estop_callback(self, msg):
        """Handle emergency stop."""
        self.e_stop = msg.data
        if self.e_stop:
            self.get_logger().warn('EMERGENCY STOP ACTIVATED')
            self.target_vel_left = 0.0
            self.target_vel_right = 0.0
    
    def control_loop(self):
        """Main control loop - send velocity commands to ODrive."""
        if not self.connected or self.e_stop:
            return
        
        try:
            # Get axis objects
            left_axis = self.odrv.axis0 if self.axis_left == 0 else self.odrv.axis1
            right_axis = self.odrv.axis0 if self.axis_right == 0 else self.odrv.axis1
            
            # Check if motors are in closed loop control
            if left_axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
                self.get_logger().warn('Left axis not in closed loop control - requesting')
                left_axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            
            if right_axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
                self.get_logger().warn('Right axis not in closed loop control - requesting')
                right_axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            
            # Send velocity commands
            left_axis.controller.input_vel = self.target_vel_left
            right_axis.controller.input_vel = self.target_vel_right
            
            # Feed watchdog if enabled in ODrive config
            if left_axis.config.enable_watchdog:
                left_axis.watchdog_feed()
            if right_axis.config.enable_watchdog:
                right_axis.watchdog_feed()
            
        except Exception as e:
            self.get_logger().error(f'Control loop error: {e}')
            self.connected = False
            # Try to reconnect
            self.connect_odrive()
    
    def publish_status(self):
        """Publish motor status and odometry."""
        if not self.connected:
            return
        
        try:
            # Get axis objects
            left_axis = self.odrv.axis0 if self.axis_left == 0 else self.odrv.axis1
            right_axis = self.odrv.axis0 if self.axis_right == 0 else self.odrv.axis1
            
            # Publish motor status
            status_msg = JointState()
            status_msg.header.stamp = self.get_clock().now().to_msg()
            status_msg.name = ['left_motor', 'right_motor']
            
            # Velocities (turns/s)
            status_msg.velocity = [
                left_axis.encoder.vel_estimate,
                right_axis.encoder.vel_estimate
            ]
            
            # Currents (Amps)
            status_msg.effort = [
                left_axis.motor.current_control.Iq_measured,
                right_axis.motor.current_control.Iq_measured
            ]
            
            # Positions (encoder counts)
            status_msg.position = [
                left_axis.encoder.pos_estimate,
                right_axis.encoder.pos_estimate
            ]
            
            # Battery voltage (V) - encode in frame_id for safety monitor
            # Format: "base_link;battery_voltage:XX.XX" to preserve TF frame
            status_msg.header.frame_id = f"base_link;battery_voltage:{self.odrv.vbus_voltage:.2f}"
            
            self.status_pub.publish(status_msg)
            
            # Compute and publish odometry
            self.compute_odometry(left_axis, right_axis)
            
            # Check for errors
            self.check_errors()
            
        except Exception as e:
            self.get_logger().error(f'Status publish error: {e}')
    
    def compute_odometry(self, left_axis, right_axis):
        """Compute wheel encoder odometry."""
        current_time = time.time()
        dt = current_time - self.last_odom_time
        
        if dt < 0.001:  # Avoid division by zero
            return
        
        # Get encoder positions (turns)
        left_pos = left_axis.encoder.pos_estimate
        right_pos = right_axis.encoder.pos_estimate
        
        # Compute delta positions (turns at motor)
        delta_left = left_pos - self.last_left_pos
        delta_right = right_pos - self.last_right_pos
        
        # Convert motor turns to wheel turns (divide by gear ratio)
        wheel_delta_left = delta_left / self.gear_ratio
        wheel_delta_right = delta_right / self.gear_ratio
        
        # Convert wheel turns to linear distance (meters)
        dist_left = wheel_delta_left * 2.0 * math.pi * self.wheel_radius
        dist_right = wheel_delta_right * 2.0 * math.pi * self.wheel_radius
        
        # Differential drive odometry
        dist_center = (dist_left + dist_right) / 2.0
        delta_theta = (dist_right - dist_left) / self.track_width
        
        # Update pose
        self.odom_x += dist_center * math.cos(self.odom_theta + delta_theta / 2.0)
        self.odom_y += dist_center * math.sin(self.odom_theta + delta_theta / 2.0)
        self.odom_theta += delta_theta
        
        # Normalize theta to [-pi, pi]
        self.odom_theta = math.atan2(math.sin(self.odom_theta), math.cos(self.odom_theta))
        
        # Compute velocities
        vel_x = dist_center / dt
        vel_theta = delta_theta / dt
        
        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Position
        odom_msg.pose.pose.position.x = self.odom_x
        odom_msg.pose.pose.position.y = self.odom_y
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from yaw)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.odom_theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.odom_theta / 2.0)
        
        # Velocity
        odom_msg.twist.twist.linear.x = vel_x
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = vel_theta
        
        self.odom_pub.publish(odom_msg)
        
        # Update state
        self.last_odom_time = current_time
        self.last_left_pos = left_pos
        self.last_right_pos = right_pos
    
    def clear_errors_callback(self, request, response):
        """Service to clear ODrive errors."""
        try:
            left_axis = self.odrv.axis0 if self.axis_left == 0 else self.odrv.axis1
            right_axis = self.odrv.axis0 if self.axis_right == 0 else self.odrv.axis1
            
            # Clear errors
            left_axis.clear_errors()
            right_axis.clear_errors()
            
            self.get_logger().info('ODrive errors cleared')
            response.success = True
            response.message = 'Errors cleared'
            
        except Exception as e:
            self.get_logger().error(f'Failed to clear errors: {e}')
            response.success = False
            response.message = str(e)
        
        return response
    
    def calibrate_callback(self, request, response):
        """Service to run motor calibration."""
        try:
            left_axis = self.odrv.axis0 if self.axis_left == 0 else self.odrv.axis1
            right_axis = self.odrv.axis0 if self.axis_right == 0 else self.odrv.axis1
            
            self.get_logger().info('Starting motor calibration...')
            
            # Request full calibration sequence
            left_axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            right_axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            
            self.get_logger().info('Calibration started - wait for completion')
            response.success = True
            response.message = 'Calibration sequence started'
            
        except Exception as e:
            self.get_logger().error(f'Failed to start calibration: {e}')
            response.success = False
            response.message = str(e)
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ODriveInterfaceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop motors before shutdown
        if node.connected:
            try:
                left_axis = node.odrv.axis0 if node.axis_left == 0 else node.odrv.axis1
                right_axis = node.odrv.axis0 if node.axis_right == 0 else node.odrv.axis1
                left_axis.controller.input_vel = 0.0
                right_axis.controller.input_vel = 0.0
                left_axis.requested_state = AXIS_STATE_IDLE
                right_axis.requested_state = AXIS_STATE_IDLE
            except:
                pass
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

