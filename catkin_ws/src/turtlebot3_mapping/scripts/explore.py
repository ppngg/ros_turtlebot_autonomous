#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import LaserScan
import math


class ExploreNode(Node):
    def __init__(self):
        super().__init__('explore_node')

        self.declare_parameter('velocity.linear', 0.3)
        self.declare_parameter('velocity.angular', 1.0)
        self.declare_parameter('obstacle_distance', 0.4)  # unit: meter, stop and turn if obstacle closer
        self.declare_parameter('safe_distance', 0.6)  #slow down if obstacle closer
        
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.timer = self.create_timer(0.05, self.update_callback)  # 20 Hz
        
        self.scan_data = None
        self.obstacle_distance = 0.4
        self.safe_distance = 0.6
        
        self.stuck_counter = 0
        self.last_front_distance = float('inf')
        self.is_reversing = False
        self.reverse_start_time = None
        self.reverse_duration = 1.0  # if stuck -> reverse for x seconds
        
        self.get_logger().info("Explore node started")
        self.get_logger().info(f"Parameters: linear={self.get_parameter('velocity.linear').value}, "
                              f"angular={self.get_parameter('velocity.angular').value}, "
                              f"obstacle_distance={self.get_parameter('obstacle_distance').value}m")
    
    def scan_callback(self, msg):
        self.scan_data = msg
        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        self.safe_distance = self.get_parameter('safe_distance').value
    
    def get_min_distance_in_sector(self, start_angle_deg, end_angle_deg):
        # in degrees, relative to robot front
        if self.scan_data is None or len(self.scan_data.ranges) == 0:
            return float('inf')
        
        num_readings = len(self.scan_data.ranges)
        angle_range = self.scan_data.angle_max - self.scan_data.angle_min
        angle_per_index = angle_range / num_readings if num_readings > 0 else 0
        
        min_dist = float('inf')
        
        for i in range(num_readings):
            # Convert index to angle
            angle_rad = self.scan_data.angle_min + i * angle_per_index
            angle_deg = angle_rad * 180.0 / math.pi
            
            # Normalize to -180 to 180 range
            if angle_deg > 180:
                angle_deg -= 360
            elif angle_deg < -180:
                angle_deg += 360
            
            # Check if angle is in the sector
            if start_angle_deg <= angle_deg <= end_angle_deg:
                distance = self.scan_data.ranges[i]
                if not math.isinf(distance) and distance > 0 and distance < self.scan_data.range_max:
                    if distance < min_dist:
                        min_dist = distance
        
        return min_dist
    
    def update_cmd_vel(self, linear, angular, linear_param=None, angular_param=None, front_dist=None, left_dist=None, right_dist=None):
        # publish velocity command
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_footprint'
        twist_stamped.twist.linear.x = linear
        twist_stamped.twist.angular.z = angular
        self.cmd_vel_pub.publish(twist_stamped)
        
        if linear_param is not None and angular_param is not None and front_dist is not None and left_dist is not None and right_dist is not None:
            self.get_logger().info(f"linear={linear_param:.1f}, angular={angular_param:.1f}, Front: {front_dist:.2f}m, Left: {left_dist:.2f}m, Right: {right_dist:.2f}m")
    
    def update_callback(self): 
        if self.scan_data is None:
            # Move forward slowly if no scan data
            linear_speed = self.get_parameter('velocity.linear').value * 0.2
            self.update_cmd_vel(linear_speed, 0.0)
            return
        
        linear_speed = self.get_parameter('velocity.linear').value
        angular_speed = self.get_parameter('velocity.angular').value
        current_time = self.get_clock().now()
        
        # divide to 3 sectors: front, left, right
        front_dist = self.get_min_distance_in_sector(-30, 30)  # Front 60 degrees
        left_dist = self.get_min_distance_in_sector(30, 90)    # Left side
        right_dist = self.get_min_distance_in_sector(-90, -30)  # Right side
        
        # Detect if stuck (front distance not changing much and below threshold)
        is_stuck = (front_dist < self.obstacle_distance and 
                   abs(front_dist - self.last_front_distance) < 0.05 and
                   front_dist < 0.45)  # Stuck if front is consistently blocked
        
        # Check if we need to reverse
        if (front_dist < self.obstacle_distance * 0.9 or is_stuck) and not self.is_reversing:
            self.is_reversing = True
            self.reverse_start_time = current_time
            # Determine which direction to turn after reverse
            if left_dist > right_dist:
                self.reverse_direction = 'LEFT'
            else:
                self.reverse_direction = 'RIGHT'
            self.get_logger().warn(f"Obstacle detected (front: {front_dist:.2f}m). Reversing and will turn {self.reverse_direction}...")
        
        # Handle reverse behavior
        if self.is_reversing:
            if self.reverse_start_time is not None:
                elapsed = (current_time - self.reverse_start_time).nanoseconds / 1e9
                if elapsed < self.reverse_duration:
                    # Reverse while turning
                    reverse_speed = linear_speed * 0.5
                    if self.reverse_direction == 'LEFT':
                        self.update_cmd_vel(-reverse_speed, angular_speed * 0.6, linear_speed, angular_speed, front_dist, left_dist, right_dist)
                    else:
                        self.update_cmd_vel(-reverse_speed, -angular_speed * 0.6, linear_speed, angular_speed, front_dist, left_dist, right_dist)
                    return
                else:
                    # Stop
                    self.is_reversing = False
                    self.reverse_start_time = None
                    self.get_logger().info(f"Finished reversing, turning {self.reverse_direction}...")
            else:
                self.is_reversing = False
        
        # Update last front distance for stuck detection
        self.last_front_distance = front_dist
        
        if front_dist < self.obstacle_distance:
            # Obstacle directly ahead -> turn away more aggressively
            if left_dist > right_dist + 0.1:
                # More space on left, turn left
                self.update_cmd_vel(0.0, angular_speed, linear_speed, angular_speed, front_dist, left_dist, right_dist)
                self.get_logger().debug(f"Obstacle ahead (front: {front_dist:.2f}m). Turning LEFT (left: {left_dist:.2f}m, right: {right_dist:.2f}m)")
            elif right_dist > left_dist + 0.1:
                # More space on right, turn right
                self.update_cmd_vel(0.0, -angular_speed, linear_speed, angular_speed, front_dist, left_dist, right_dist)
                self.get_logger().debug(f"Obstacle ahead (front: {front_dist:.2f}m). Turning RIGHT (left: {left_dist:.2f}m, right: {right_dist:.2f}m)")
            else:
                # Very close distances -> turn in place more aggressively
                if left_dist > right_dist:
                    self.update_cmd_vel(0.0, angular_speed * 1.2, linear_speed, angular_speed, front_dist, left_dist, right_dist)
                else:
                    self.update_cmd_vel(0.0, -angular_speed * 1.2, linear_speed, angular_speed, front_dist, left_dist, right_dist)
        
        elif front_dist < self.safe_distance:
            # Getting close to obstacle -> slow down and turn slightly
            if left_dist > right_dist + 0.1:
                # More space on left -> turn left slightly while moving
                self.update_cmd_vel(linear_speed * 0.4, angular_speed * 0.6, linear_speed, angular_speed, front_dist, left_dist, right_dist)
            elif right_dist > left_dist + 0.1:
                # More space on right -> turn right slightly while moving
                self.update_cmd_vel(linear_speed * 0.4, -angular_speed * 0.6, linear_speed, angular_speed, front_dist, left_dist, right_dist)
            else:
                # Similar distances -> slow down more
                self.update_cmd_vel(linear_speed * 0.3, 0.0, linear_speed, angular_speed, front_dist, left_dist, right_dist)
        
        elif left_dist < self.obstacle_distance * 1.2:
            # Obstacle on left -> follow wall while turning right slightly
            self.update_cmd_vel(linear_speed, -angular_speed * 0.3, linear_speed, angular_speed, front_dist, left_dist, right_dist)
        
        elif right_dist < self.obstacle_distance * 1.2:
            # obstacle on right
            self.update_cmd_vel(linear_speed, angular_speed * 0.3, linear_speed, angular_speed, front_dist, left_dist, right_dist)
        
        else:
            # Clear path
            self.update_cmd_vel(linear_speed, 0.0, linear_speed, angular_speed, front_dist, left_dist, right_dist)
            # Reset stuck counter when moving forward
            self.stuck_counter = 0
        


def main(args=None):
    rclpy.init(args=args)
    
    explorer = None
    try:
        explorer = ExploreNode()
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        pass
    finally:
        if explorer is not None:
            explorer.update_cmd_vel(0.0, 0.0)
            explorer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
