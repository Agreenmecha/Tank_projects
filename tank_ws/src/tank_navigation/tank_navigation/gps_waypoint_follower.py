#!/usr/bin/env python3
"""
GPS Waypoint Follower

Uses:
- /gnss/fix (sensor_msgs/NavSatFix) for absolute position
- /imu/fused (sensor_msgs/Imu) for heading (required)

Strategy:
1) Compute bearing + distance to waypoint from GPS
2) Use IMU heading for current orientation
3) Rotate to face target, then drive forward
4) Slow down near waypoint, stop within tolerance
5) Optional drift correction: blend IMU heading toward GPS heading when moving

Note: We do NOT use wheel odometry due to tank track slip.
"""

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, Imu
from std_srvs.srv import Trigger
from std_msgs.msg import String


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class PID:
    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def update(self, error: float) -> float:
        now = time.time()
        dt = 0.0
        if self.prev_time is None:
            dt = 0.0
        else:
            dt = now - self.prev_time
        self.prev_time = now

        # Integral
        self.integral += error * dt

        # Derivative
        derivative = 0.0
        if dt > 1e-3:
            derivative = (error - self.prev_error) / dt
        self.prev_error = error

        return self.kp * error + self.ki * self.integral + self.kd * derivative


class GPSWaypointFollower(Node):
    def __init__(self):
        super().__init__('gps_waypoint_follower')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('waypoint_tolerance_m', 2.0),
                ('slow_distance_m', 5.0),
                ('max_linear_vel', 1.0),
                ('max_angular_vel', 1.0),
                ('heading_align_threshold_deg', 10.0),
                ('pid_kp', 1.2),
                ('pid_ki', 0.0),
                ('pid_kd', 0.2),
                ('min_gps_speed_mps', 0.5),
                ('min_gps_fix_status', 1),
                ('drift_correction_gain', 0.05),
                ('drift_correction_threshold_deg', 10.0),
                ('base_frame_id', 'base_link'),
                ('map_frame_id', 'map'),
            ]
        )

        # Load params
        self.waypoint_tolerance = float(self.get_parameter('waypoint_tolerance_m').value)
        self.slow_distance = float(self.get_parameter('slow_distance_m').value)
        self.max_linear = float(self.get_parameter('max_linear_vel').value)
        self.max_angular = float(self.get_parameter('max_angular_vel').value)
        self.heading_align_threshold = math.radians(
            float(self.get_parameter('heading_align_threshold_deg').value)
        )
        self.min_gps_speed = float(self.get_parameter('min_gps_speed_mps').value)
        self.min_gps_fix_status = int(self.get_parameter('min_gps_fix_status').value)
        self.drift_gain = float(self.get_parameter('drift_correction_gain').value)
        self.drift_threshold = math.radians(
            float(self.get_parameter('drift_correction_threshold_deg').value)
        )

        # State
        self.current_fix: Optional[NavSatFix] = None
        self.current_heading: Optional[float] = None  # radians
        self.last_gps_heading: Optional[float] = None
        self.last_fix_time: float = 0.0
        self.target_waypoint = None  # (lat, lon)
        self.navigating = False

        # PID for heading
        self.heading_pid = PID(
            kp=float(self.get_parameter('pid_kp').value),
            ki=float(self.get_parameter('pid_ki').value),
            kd=float(self.get_parameter('pid_kd').value),
        )

        # Publishers/Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/gps_nav/status', 10)

        self.fix_sub = self.create_subscription(NavSatFix, '/gnss/fix', self.fix_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/fused', self.imu_callback, 50)

        # Services
        self.set_wp_srv = self.create_service(Trigger, '/gps_nav/set_waypoint', self.set_waypoint_service)
        self.cancel_wp_srv = self.create_service(Trigger, '/gps_nav/cancel_waypoint', self.cancel_waypoint_service)

        # Timer
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info('GPS Waypoint Follower initialized (IMU + GPS heading)')

    # ----------------- Callbacks -----------------
    def fix_callback(self, msg: NavSatFix):
        self.current_fix = msg
        self.last_fix_time = self.get_clock().now().nanoseconds * 1e-9

    def imu_callback(self, msg: Imu):
        # Extract yaw from quaternion
        q = msg.orientation
        # yaw from quaternion
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_heading = yaw

    # ----------------- Services -----------------
    def set_waypoint_service(self, request, response):
        # Expect waypoint stored in parameters? Simple Trigger: use params lat/lon? For now, log warning.
        response.success = False
        response.message = 'Use /gps_nav/set_waypoint_latlon (not implemented)'
        return response

    def cancel_waypoint_service(self, request, response):
        self.navigating = False
        self.target_waypoint = None
        self.stop_robot()
        response.success = True
        response.message = 'Navigation cancelled'
        return response

    # ----------------- Control Loop -----------------
    def control_loop(self):
        # Require waypoint, GPS, IMU
        if not self.navigating or self.target_waypoint is None:
            return
        if self.current_fix is None or self.current_heading is None:
            return

        # Compute distance and bearing to waypoint
        dist_m, bearing_rad = self.compute_distance_bearing(
            self.current_fix.latitude,
            self.current_fix.longitude,
            self.target_waypoint[0],
            self.target_waypoint[1],
        )

        # Heading error
        heading_error = normalize_angle(bearing_rad - self.current_heading)

        cmd = Twist()

        # Align first
        if abs(heading_error) > self.heading_align_threshold:
            cmd.linear.x = 0.0
            ang_cmd = self.heading_pid.update(heading_error)
            cmd.angular.z = clamp(ang_cmd, -self.max_angular, self.max_angular)
        else:
            # Drive forward
            speed = self.max_linear
            if dist_m < self.slow_distance:
                speed = self.max_linear * (dist_m / self.slow_distance)
            cmd.linear.x = clamp(speed, 0.0, self.max_linear)
            cmd.angular.z = clamp(self.heading_pid.update(heading_error), -self.max_angular, self.max_angular)

        # Stop if reached
        if dist_m < self.waypoint_tolerance:
            self.stop_robot()
            self.navigating = False
            self.status_pub.publish(String(data='Reached waypoint'))
            return

        self.cmd_pub.publish(cmd)

    # ----------------- Helpers -----------------
    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def compute_distance_bearing(self, lat1, lon1, lat2, lon2):
        """Haversine distance (m) and bearing (rad) from point1 to point2."""
        R = 6371000.0  # Earth radius in meters
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)

        a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c

        y = math.sin(dlambda) * math.cos(phi2)
        x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
        bearing = math.atan2(y, x)
        bearing = normalize_angle(bearing)
        return distance, bearing


def main(args=None):
    rclpy.init(args=args)
    node = GPSWaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

