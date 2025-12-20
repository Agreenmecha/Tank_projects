#!/usr/bin/env python3
"""
GPS Waypoint Manager
Converts GPS waypoints (lat/lon) to map coordinates and sends to Nav2
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoint
from ublox_ubx_msgs.msg import UBXNavHPPosLLH
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from std_msgs.msg import String
import json
import math
from typing import List, Tuple


class GPSWaypointManager(Node):
    """
    Manages GPS waypoint missions
    - Receives GPS waypoints (lat/lon)
    - Converts to map coordinates (x/y)
    - Sends to Nav2 for autonomous navigation
    """
    
    def __init__(self):
        super().__init__('gps_waypoint_manager')
        
        # Parameters
        self.declare_parameter('datum_lat', 0.0)  # Will be set from first GPS fix
        self.declare_parameter('datum_lon', 0.0)
        self.declare_parameter('datum_alt', 0.0)
        
        # GPS datum (origin point in GPS coordinates)
        self.datum_lat = None
        self.datum_lon = None
        self.datum_set = False
        
        # Current GPS position
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.gps_valid = False
        
        # Subscribers
        self.gps_sub = self.create_subscription(
            UBXNavHPPosLLH,
            '/ubx_nav_hp_pos_llh',
            self.gps_callback,
            10
        )
        
        # Waypoint input (JSON format)
        self.waypoint_sub = self.create_subscription(
            String,
            '/gps_waypoints',
            self.waypoints_callback,
            10
        )
        
        # Publishers
        self.status_pub = self.create_publisher(
            String,
            '/gps_mission_status',
            10
        )
        
        # Nav2 action clients
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        self.follow_waypoints_client = ActionClient(
            self,
            FollowWaypoints,
            'follow_waypoints'
        )
        
        self.get_logger().info('GPS Waypoint Manager started')
        self.get_logger().info('Waiting for GPS fix to set datum...')
    
    def gps_callback(self, msg: UBXNavHPPosLLH):
        """Store current GPS position and set datum if needed"""
        # Convert ublox format: lat/lon in deg * 1e-7, hp in deg * 1e-9
        if not msg.invalid_lat and not msg.invalid_lon:
            self.current_lat = (msg.lat + msg.lat_hp * 0.01) * 1e-7
            self.current_lon = (msg.lon + msg.lon_hp * 0.01) * 1e-7
            self.gps_valid = True
            
            # Set datum from first GPS fix if not already set
            if not self.datum_set:
                self.datum_lat = self.current_lat
                self.datum_lon = self.current_lon
                self.datum_set = True
                self.get_logger().info(
                    f'GPS Datum set: lat={self.datum_lat:.8f}, '
                    f'lon={self.datum_lon:.8f}'
                )
        else:
            self.gps_valid = False
    
    def gps_to_map(self, lat: float, lon: float) -> Tuple[float, float]:
        """
        Convert GPS coordinates to local map coordinates
        Uses simple equirectangular projection (good for small areas)
        
        Returns: (x, y) in meters from datum
        """
        if not self.datum_set:
            self.get_logger().error('GPS datum not set! Cannot convert coordinates.')
            return (0.0, 0.0)
        
        # Earth radius in meters
        R = 6378137.0
        
        # Convert to radians
        lat1 = math.radians(self.datum_lat)
        lon1 = math.radians(self.datum_lon)
        lat2 = math.radians(lat)
        lon2 = math.radians(lon)
        
        # Calculate distances
        x = R * (lon2 - lon1) * math.cos((lat1 + lat2) / 2.0)
        y = R * (lat2 - lat1)
        
        return (x, y)
    
    def map_to_gps(self, x: float, y: float) -> Tuple[float, float]:
        """
        Convert map coordinates back to GPS
        Inverse of gps_to_map()
        
        Returns: (latitude, longitude)
        """
        if not self.datum_set:
            return (0.0, 0.0)
        
        R = 6378137.0
        
        lat1 = math.radians(self.datum_lat)
        lon1 = math.radians(self.datum_lon)
        
        lat2 = lat1 + (y / R)
        lon2 = lon1 + (x / R) / math.cos((lat1 + lat2) / 2.0)
        
        return (math.degrees(lat2), math.degrees(lon2))
    
    def waypoints_callback(self, msg: String):
        """
        Receive GPS waypoints via JSON
        
        Format:
        {
            "waypoints": [
                {"lat": 37.1234, "lon": -122.5678, "name": "Point A"},
                {"lat": 37.1235, "lon": -122.5679, "name": "Point B"}
            ]
        }
        """
        try:
            data = json.loads(msg.data)
            waypoints = data.get('waypoints', [])
            
            if not waypoints:
                self.get_logger().warn('No waypoints in message')
                return
            
            self.get_logger().info(f'Received {len(waypoints)} GPS waypoints')
            
            # Convert GPS to map coordinates
            poses = []
            for i, wp in enumerate(waypoints):
                lat = wp.get('lat')
                lon = wp.get('lon')
                name = wp.get('name', f'Waypoint {i+1}')
                
                if lat is None or lon is None:
                    self.get_logger().warn(f'Invalid waypoint {i}: missing lat/lon')
                    continue
                
                # Convert to map coordinates
                x, y = self.gps_to_map(lat, lon)
                
                # Create PoseStamped
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0
                # Orientation: face next waypoint (will be calculated)
                pose.pose.orientation.w = 1.0
                
                poses.append(pose)
                
                self.get_logger().info(
                    f'{name}: GPS({lat:.6f}, {lon:.6f}) → '
                    f'Map({x:.2f}m, {y:.2f}m)'
                )
            
            if len(poses) == 1:
                # Single waypoint - use NavigateToPose
                self.navigate_to_pose(poses[0])
            elif len(poses) > 1:
                # Multiple waypoints - use FollowWaypoints
                self.follow_waypoints(poses)
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing waypoints: {e}')
    
    def navigate_to_pose(self, pose: PoseStamped):
        """Navigate to single GPS waypoint"""
        self.get_logger().info('Sending single waypoint to Nav2...')
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        self.nav_to_pose_client.wait_for_server()
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        send_goal_future.add_done_callback(self.nav_goal_response_callback)
    
    def follow_waypoints(self, poses: List[PoseStamped]):
        """Navigate through multiple GPS waypoints"""
        self.get_logger().info(f'Sending {len(poses)} waypoints to Nav2...')
        
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses
        
        self.follow_waypoints_client.wait_for_server()
        send_goal_future = self.follow_waypoints_client.send_goal_async(
            goal_msg,
            feedback_callback=self.waypoints_feedback_callback
        )
        send_goal_future.add_done_callback(self.waypoints_goal_response_callback)
    
    def nav_feedback_callback(self, feedback_msg):
        """Handle NavigateToPose feedback"""
        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining
        self.get_logger().info(f'Distance remaining: {distance:.2f}m')
        
        # Publish status
        status = json.dumps({
            'type': 'navigate_to_pose',
            'distance_remaining': distance
        })
        self.status_pub.publish(String(data=status))
    
    def waypoints_feedback_callback(self, feedback_msg):
        """Handle FollowWaypoints feedback"""
        feedback = feedback_msg.feedback
        current = feedback.current_waypoint
        self.get_logger().info(f'At waypoint {current + 1}')
        
        # Publish status
        status = json.dumps({
            'type': 'follow_waypoints',
            'current_waypoint': current
        })
        self.status_pub.publish(String(data=status))
    
    def nav_goal_response_callback(self, future):
        """Handle NavigateToPose goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
        
        self.get_logger().info('Goal accepted! Navigating...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)
    
    def waypoints_goal_response_callback(self, future):
        """Handle FollowWaypoints goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Waypoints rejected!')
            return
        
        self.get_logger().info('Waypoints accepted! Starting mission...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.waypoints_result_callback)
    
    def nav_result_callback(self, future):
        """Handle NavigateToPose result"""
        result = future.result().result
        self.get_logger().info('Navigation complete!')
        
        status = json.dumps({'type': 'complete', 'success': True})
        self.status_pub.publish(String(data=status))
    
    def waypoints_result_callback(self, future):
        """Handle FollowWaypoints result"""
        result = future.result().result
        missed = result.missed_waypoints
        
        if len(missed) == 0:
            self.get_logger().info('All waypoints reached!')
        else:
            self.get_logger().warn(f'Missed {len(missed)} waypoints: {missed}')
        
        status = json.dumps({
            'type': 'complete',
            'success': len(missed) == 0,
            'missed_waypoints': missed
        })
        self.status_pub.publish(String(data=status))


def main(args=None):
    rclpy.init(args=args)
    node = GPSWaypointManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

