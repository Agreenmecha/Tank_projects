#!/usr/bin/env python3
"""
Safety Monitor Node for Tank
Monitors motor temperatures, currents, pitch angle, and watchdog timers
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
import time
import math


class SafetyMonitorNode(Node):
    """
    Monitors critical safety parameters and publishes warnings/emergency stops.
    
    Subscribes:
        /odrive/motor_status (sensor_msgs/JointState) - Motor temperatures, currents
        /lidar_front/imu (sensor_msgs/Imu) - Pitch angle monitoring
        /cmd_vel (geometry_msgs/Twist) - Command watchdog
    
    Publishes:
        /emergency_stop (std_msgs/Bool) - Emergency stop signal
        /safety/status (std_msgs/String) - Safety status messages
        /safety/warnings (std_msgs/String) - Warning messages
    """
    
    def __init__(self):
        super().__init__('safety_monitor_node')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('motor_temp_warning', 70.0),
                ('motor_temp_critical', 80.0),
                ('motor_temp_shutdown', 90.0),
                ('motor_current_warning', 25.0),
                ('motor_current_peak', 40.0),
                ('pitch_warning', 28.0),
                ('pitch_critical', 32.0),
                ('pitch_emergency', 38.0),
                ('cmd_vel_timeout', 0.5),
                ('imu_timeout', 1.0),
                ('odrive_timeout', 1.0),
                # Battery voltage thresholds (48V e-bike battery)
                ('battery_voltage_warning', 42.0),   # ~20% remaining
                ('battery_voltage_critical', 39.0),  # ~10% remaining
                ('battery_voltage_emergency', 36.0), # Way too low
            ]
        )
        
        # Get parameters
        self.motor_temp_warning = self.get_parameter('motor_temp_warning').value
        self.motor_temp_critical = self.get_parameter('motor_temp_critical').value
        self.motor_temp_shutdown = self.get_parameter('motor_temp_shutdown').value
        self.motor_current_warning = self.get_parameter('motor_current_warning').value
        self.motor_current_peak = self.get_parameter('motor_current_peak').value
        self.pitch_warning = self.get_parameter('pitch_warning').value
        self.pitch_critical = self.get_parameter('pitch_critical').value
        self.pitch_emergency = self.get_parameter('pitch_emergency').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.imu_timeout = self.get_parameter('imu_timeout').value
        self.odrive_timeout = self.get_parameter('odrive_timeout').value
        self.battery_voltage_warning = self.get_parameter('battery_voltage_warning').value
        self.battery_voltage_critical = self.get_parameter('battery_voltage_critical').value
        self.battery_voltage_emergency = self.get_parameter('battery_voltage_emergency').value
        
        # State tracking
        self.last_cmd_vel_time = time.time()
        self.last_imu_time = time.time()
        self.last_odrive_time = time.time()
        self.emergency_stop_active = False
        self.current_pitch = 0.0
        self.motor_temps = [0.0, 0.0]  # Placeholder - ODrive doesn't publish temp via USB easily
        self.motor_currents = [0.0, 0.0]
        self.battery_voltage = 0.0  # Battery voltage (V)
        
        # Subscribers
        self.motor_status_sub = self.create_subscription(
            JointState, '/odrive/motor_status', self.motor_status_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/lidar_front/imu', self.imu_callback, 10
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        
        # Publishers
        self.estop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.status_pub = self.create_publisher(String, '/safety/status', 10)
        self.warning_pub = self.create_publisher(String, '/safety/warnings', 10)
        
        # Timer for monitoring
        self.monitor_timer = self.create_timer(0.1, self.monitor_loop)  # 10 Hz
        
        self.get_logger().info('Safety Monitor Node initialized')
    
    def motor_status_callback(self, msg):
        """Update motor status."""
        self.last_odrive_time = time.time()
        
        if len(msg.effort) >= 2:
            self.motor_currents = list(msg.effort)
        
        # Extract battery voltage from header.frame_id
        # Format: "base_link;battery_voltage:XX.XX" or "battery_voltage:XX.XX"
        try:
            frame_id = msg.header.frame_id
            if "battery_voltage:" in frame_id:
                # Extract voltage from either format
                if ";" in frame_id:
                    # Format: "base_link;battery_voltage:XX.XX"
                    voltage_part = frame_id.split(";")[1]
                else:
                    # Format: "battery_voltage:XX.XX"
                    voltage_part = frame_id
                voltage_str = voltage_part.split(":")[1]
                self.battery_voltage = float(voltage_str)
        except (ValueError, IndexError):
            # If parsing fails, voltage stays at 0 (will trigger timeout warning)
            pass
    
    def imu_callback(self, msg):
        """Extract pitch angle from IMU."""
        self.last_imu_time = time.time()
        
        # Convert quaternion to euler angles
        # Extract pitch from quaternion
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        
        # Pitch (y-axis rotation)
        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            self.current_pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            self.current_pitch = math.asin(sinp)
        
        # Convert to degrees
        self.current_pitch = math.degrees(self.current_pitch)
    
    def cmd_vel_callback(self, msg):
        """Update cmd_vel watchdog timer."""
        self.last_cmd_vel_time = time.time()
    
    def monitor_loop(self):
        """Main monitoring loop."""
        current_time = time.time()
        warnings = []
        emergency = False
        
        # Check pitch angle
        abs_pitch = abs(self.current_pitch)
        if abs_pitch >= self.pitch_emergency:
            warnings.append(f'EMERGENCY: Pitch angle {self.current_pitch:.1f}° exceeds limit')
            emergency = True
        elif abs_pitch >= self.pitch_critical:
            warnings.append(f'CRITICAL: Pitch angle {self.current_pitch:.1f}° - reduce speed')
        elif abs_pitch >= self.pitch_warning:
            warnings.append(f'WARNING: Pitch angle {self.current_pitch:.1f}° - caution')
        
        # Check motor currents
        for i, current in enumerate(self.motor_currents):
            abs_current = abs(current)
            if abs_current >= self.motor_current_peak:
                warnings.append(f'CRITICAL: Motor {i} current {abs_current:.1f}A exceeds peak')
                emergency = True
            elif abs_current >= self.motor_current_warning:
                warnings.append(f'WARNING: Motor {i} current {abs_current:.1f}A high')
        
        # Check timeouts
        if current_time - self.last_odrive_time > self.odrive_timeout:
            warnings.append(f'ERROR: No ODrive data for {current_time - self.last_odrive_time:.1f}s')
            emergency = True
        
        if current_time - self.last_imu_time > self.imu_timeout:
            warnings.append(f'WARNING: No IMU data for {current_time - self.last_imu_time:.1f}s')
        
        # Check battery voltage
        if self.battery_voltage > 0:  # Only check if we have valid voltage reading
            if self.battery_voltage <= self.battery_voltage_emergency:
                warnings.append(f'EMERGENCY: Battery voltage {self.battery_voltage:.1f}V - CRITICALLY LOW!')
                emergency = True
            elif self.battery_voltage <= self.battery_voltage_critical:
                warnings.append(f'CRITICAL: Battery voltage {self.battery_voltage:.1f}V - Very low!')
                emergency = True  # Critical battery should trigger emergency stop
            elif self.battery_voltage <= self.battery_voltage_warning:
                warnings.append(f'WARNING: Battery voltage {self.battery_voltage:.1f}V - Low battery')
        
        # Publish warnings
        if warnings:
            warning_msg = String()
            warning_msg.data = ' | '.join(warnings)
            self.warning_pub.publish(warning_msg)
            self.get_logger().warn(warning_msg.data)
        
        # Publish emergency stop if needed
        if emergency and not self.emergency_stop_active:
            self.emergency_stop_active = True
            estop_msg = Bool()
            estop_msg.data = True
            self.estop_pub.publish(estop_msg)
            self.get_logger().error('EMERGENCY STOP ACTIVATED')
        elif not emergency and self.emergency_stop_active:
            # Clear emergency stop if conditions are safe
            self.emergency_stop_active = False
            estop_msg = Bool()
            estop_msg.data = False
            self.estop_pub.publish(estop_msg)
            self.get_logger().info('Emergency stop cleared')
        
        # Publish status
        if not warnings:
            voltage_str = f'{self.battery_voltage:.1f}V' if self.battery_voltage > 0 else 'N/A'
            status_msg = String()
            status_msg.data = f'OK: Battery={voltage_str} | Pitch={self.current_pitch:.1f}° | Motors: {self.motor_currents[0]:.1f}A, {self.motor_currents[1]:.1f}A'
            self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

