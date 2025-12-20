#!/usr/bin/env python3
"""
Convert ublox UBXNavPVT messages to standard sensor_msgs/NavSatFix
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from ublox_ubx_msgs.msg import UBXNavPVT


class UbxToNavSatFix(Node):
    def __init__(self):
        super().__init__('ubx_to_navsat_fix')
        
        self.pub = self.create_publisher(NavSatFix, '/fix', 10)
        self.sub = self.create_subscription(
            UBXNavPVT,
            '/ubx_nav_pvt',
            self.pvt_callback,
            10
        )
        
        self.get_logger().info('UBX to NavSatFix converter started')
    
    def pvt_callback(self, msg: UBXNavPVT):
        """Convert UBXNavPVT to NavSatFix"""
        fix = NavSatFix()
        
        # Header
        fix.header.stamp = msg.header.stamp
        fix.header.frame_id = 'gnss'
        
        # Status
        fix.status.status = NavSatStatus.STATUS_FIX if msg.fix_type >= 2 else NavSatStatus.STATUS_NO_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        
        # Position (convert from deg * 1e-7 to degrees)
        fix.latitude = msg.lat * 1e-7
        fix.longitude = msg.lon * 1e-7
        fix.altitude = msg.height * 1e-3  # mm to meters
        
        # Covariance (h_acc and v_acc are in mm)
        h_acc_m = msg.h_acc * 1e-3  # horizontal accuracy in meters
        v_acc_m = msg.v_acc * 1e-3  # vertical accuracy in meters
        
        fix.position_covariance[0] = h_acc_m ** 2  # East variance
        fix.position_covariance[4] = h_acc_m ** 2  # North variance
        fix.position_covariance[8] = v_acc_m ** 2  # Up variance
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        
        self.pub.publish(fix)


def main(args=None):
    rclpy.init(args=args)
    node = UbxToNavSatFix()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

