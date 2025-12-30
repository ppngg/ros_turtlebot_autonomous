#!/usr/bin/env python3

"""
Frontier-based exploration node.
The robot navigates to frontiers (boundaries between known and unknown space)
to efficiently explore the environment and stops when exploration is complete.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math
from collections import deque


class FrontierExploreNode(Node):
    def __init__(self):
        super().__init__('frontier_explore_node')
        
        # Declare parameters
        self.declare_parameter('velocity.linear', 0.3)
        self.declare_parameter('velocity.angular', 1.0)
        self.declare_parameter('obstacle_distance', 0.4)
        self.declare_parameter('safe_distance', 0.6)
        self.declare_parameter('frontier_search_radius', 2.0)  # Search for frontiers within this radius
        self.declare_parameter('min_frontier_size', 3)  # Minimum number of cells in a frontier
        self.declare_parameter('exploration_timeout', 300.0)  # Stop after 5 minutes if no progress
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        # TF buffer and listener for robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Timer for control loop (20 Hz)
        self.timer = self.create_timer(0.05, self.update_callback)
        
        # Timer for frontier detection (1 Hz - less frequent)
        self.frontier_timer = self.create_timer(1.0, self.update_frontiers)
        
        # State variables
        self.scan_data = None
        self.map_data = None
        self.map_info = None
        self.robot_pose = None  # (x, y, yaw) in map frame
        
        # Frontier exploration state
        self.frontiers = []  # List of frontier points [(x, y), ...]
        self.current_goal = None  # Current frontier goal (x, y)
        self.goal_reached_threshold = 0.5  # Consider goal reached if within 0.5m
        self.exploration_complete = False
        self.last_frontier_update = None
        self.start_time = self.get_clock().now()
        
        # Obstacle avoidance state
        self.is_reversing = False
        self.reverse_start_time = None
        self.reverse_duration = 1.0
        self.last_front_distance = float('inf')
        
        # Visited areas tracking (simple grid-based)
        self.visited_cells = set()
        self.cell_size = 0.5  # Track visited areas in 0.5m cells
        
        self.get_logger().info("Frontier-based exploration node started")
        self.get_logger().info("Robot will navigate to unexplored frontiers and stop when exploration is complete")
    
    def scan_callback(self, msg):
        """Callback for laser scan data"""
        self.scan_data = msg
    
    def map_callback(self, msg):
        """Callback for occupancy grid map"""
        self.map_data = msg
        self.map_info = msg.info
    
    def get_robot_pose(self):
        """Get robot pose in map frame using TF"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time()
            )
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            # Extract yaw from quaternion
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            
            yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
            
            self.robot_pose = (x, y, yaw)
            return True
        except TransformException as e:
            # self.get_logger().debug(f"Could not get robot pose: {e}")
            return False
    
    def world_to_map(self, wx, wy):
        """Convert world coordinates to map cell indices"""
        if self.map_info is None:
            return None
        
        mx = int((wx - self.map_info.origin.position.x) / self.map_info.resolution)
        my = int((wy - self.map_info.origin.position.y) / self.map_info.resolution)
        
        if 0 <= mx < self.map_info.width and 0 <= my < self.map_info.height:
            return (mx, my)
        return None
    
    def map_to_world(self, mx, my):
        """Convert map cell indices to world coordinates"""
        if self.map_info is None:
            return None
        
        wx = mx * self.map_info.resolution + self.map_info.origin.position.x
        wy = my * self.map_info.resolution + self.map_info.origin.position.y
        return (wx, wy)
    
    def get_map_cell(self, mx, my):
        """Get occupancy value at map cell (mx, my)"""
        if self.map_data is None or self.map_info is None:
            return -1
        
        if not (0 <= mx < self.map_info.width and 0 <= my < self.map_info.height):
            return -1
        
        index = my * self.map_info.width + mx
        if 0 <= index < len(self.map_data.data):
            return self.map_data.data[index]
        return -1
    
    def is_frontier_cell(self, mx, my):
        """Check if a cell is a frontier (free space adjacent to unknown)"""
        # Cell must be free space
        if self.get_map_cell(mx, my) != 0:
            return False
        
        # Check 8 neighbors for unknown cells
        neighbors = [
            (mx-1, my-1), (mx, my-1), (mx+1, my-1),
            (mx-1, my),               (mx+1, my),
            (mx-1, my+1), (mx, my+1), (mx+1, my+1)
        ]
        
        for nx, ny in neighbors:
            if self.get_map_cell(nx, ny) == -1:  # Unknown
                return True
        
        return False
    
    def find_frontiers(self):
        """Find all frontier cells in the map"""
        if self.map_data is None or self.map_info is None:
            return []
        
        frontiers = []
        min_frontier_size = self.get_parameter('min_frontier_size').value
        
        # Only search near robot to reduce computation
        if self.robot_pose is None:
            return []
        
        rx, ry, _ = self.robot_pose
        search_radius = self.get_parameter('frontier_search_radius').value
        
        # Calculate search bounds in map coordinates
        min_x = max(0, int((rx - search_radius - self.map_info.origin.position.x) / self.map_info.resolution))
        max_x = min(self.map_info.width, int((rx + search_radius - self.map_info.origin.position.x) / self.map_info.resolution))
        min_y = max(0, int((ry - search_radius - self.map_info.origin.position.y) / self.map_info.resolution))
        max_y = min(self.map_info.height, int((ry + search_radius - self.map_info.origin.position.y) / self.map_info.resolution))
        
        # Find frontier cells
        frontier_cells = []
        for mx in range(min_x, max_x):
            for my in range(min_y, max_y):
                if self.is_frontier_cell(mx, my):
                    frontier_cells.append((mx, my))
        
        # Group frontier cells into clusters
        if not frontier_cells:
            return []
        
        # Simple clustering: group cells within 3 cells of each other
        clusters = []
        visited = set()
        
        for cell in frontier_cells:
            if cell in visited:
                continue
            
            cluster = []
            queue = deque([cell])
            visited.add(cell)
            
            while queue:
                current = queue.popleft()
                cluster.append(current)
                
                # Check neighbors
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        neighbor = (current[0] + dx, current[1] + dy)
                        if neighbor in frontier_cells and neighbor not in visited:
                            visited.add(neighbor)
                            queue.append(neighbor)
            
            if len(cluster) >= min_frontier_size:
                # Calculate cluster center
                cx = sum(c[0] for c in cluster) / len(cluster)
                cy = sum(c[1] for c in cluster) / len(cluster)
                center_world = self.map_to_world(int(cx), int(cy))
                if center_world:
                    clusters.append(center_world)
        
        return clusters
    
    def select_best_frontier(self, frontiers):
        """Select the best frontier to navigate to"""
        if not frontiers or self.robot_pose is None:
            return None
        
        rx, ry, _ = self.robot_pose
        best_frontier = None
        best_score = float('-inf')
        
        for fx, fy in frontiers:
            # Calculate distance
            dist = math.sqrt((fx - rx)**2 + (fy - ry)**2)
            
            # Skip if too close (already explored) or too far
            if dist < 0.3 or dist > 5.0:
                continue
            
            # Check if we've visited this area recently
            visited_key = (int(fx / self.cell_size), int(fy / self.cell_size))
            if visited_key in self.visited_cells:
                continue
            
            # Score: prefer closer frontiers, but not too close
            score = 1.0 / (dist + 0.1)  # Inverse distance
            
            if score > best_score:
                best_score = score
                best_frontier = (fx, fy)
        
        return best_frontier
    
    def update_frontiers(self):
        """Update frontier list (called periodically)"""
        if not self.get_robot_pose():
            return
        
        # Find frontiers
        frontiers = self.find_frontiers()
        self.frontiers = frontiers
        
        # Select best frontier if we don't have a current goal or goal is reached
        if self.current_goal is None or self.is_goal_reached():
            self.current_goal = self.select_best_frontier(frontiers)
            
            if self.current_goal:
                self.get_logger().info(f"New frontier goal: ({self.current_goal[0]:.2f}, {self.current_goal[1]:.2f})")
                # Mark area as visited
                visited_key = (int(self.current_goal[0] / self.cell_size), 
                             int(self.current_goal[1] / self.cell_size))
                self.visited_cells.add(visited_key)
            else:
                # Check if exploration is complete
                if len(frontiers) == 0:
                    if not self.exploration_complete:
                        self.exploration_complete = True
                        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
                        self.get_logger().info(f"Exploration complete! No more frontiers found. Time: {elapsed:.1f}s")
                else:
                    self.get_logger().warn(f"No suitable frontier found. {len(frontiers)} frontiers available but all visited or too close/far")
        
        self.last_frontier_update = self.get_clock().now()
    
    def is_goal_reached(self):
        """Check if current goal has been reached"""
        if self.current_goal is None or self.robot_pose is None:
            return False
        
        rx, ry, _ = self.robot_pose
        gx, gy = self.current_goal
        
        dist = math.sqrt((gx - rx)**2 + (gy - ry)**2)
        return dist < self.goal_reached_threshold
    
    def get_min_distance_in_sector(self, start_angle_deg, end_angle_deg):
        """Get minimum distance in a sector"""
        if self.scan_data is None or len(self.scan_data.ranges) == 0:
            return float('inf')
        
        num_readings = len(self.scan_data.ranges)
        angle_range = self.scan_data.angle_max - self.scan_data.angle_min
        angle_per_index = angle_range / num_readings if num_readings > 0 else 0
        
        min_dist = float('inf')
        
        for i in range(num_readings):
            angle_rad = self.scan_data.angle_min + i * angle_per_index
            angle_deg = angle_rad * 180.0 / math.pi
            
            if angle_deg > 180:
                angle_deg -= 360
            elif angle_deg < -180:
                angle_deg += 360
            
            if start_angle_deg <= angle_deg <= end_angle_deg:
                distance = self.scan_data.ranges[i]
                if not math.isinf(distance) and distance > 0 and distance < self.scan_data.range_max:
                    if distance < min_dist:
                        min_dist = distance
        
        return min_dist
    
    def calculate_angle_to_goal(self):
        """Calculate angle from robot to current goal"""
        if self.current_goal is None or self.robot_pose is None:
            return None
        
        rx, ry, ryaw = self.robot_pose
        gx, gy = self.current_goal
        
        dx = gx - rx
        dy = gy - ry
        
        goal_angle = math.atan2(dy, dx)
        angle_diff = goal_angle - ryaw
        
        # Normalize to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        return angle_diff
    
    def update_cmd_vel(self, linear, angular):
        """Publish velocity command"""
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_footprint'
        twist_stamped.twist.linear.x = linear
        twist_stamped.twist.angular.z = angular
        self.cmd_vel_pub.publish(twist_stamped)
    
    def update_callback(self):
        """Main control loop"""
        # Stop if exploration is complete
        if self.exploration_complete:
            self.update_cmd_vel(0.0, 0.0)
            return
        
        # Get robot pose
        if not self.get_robot_pose():
            # No pose available, use simple obstacle avoidance
            if self.scan_data is None:
                return
            
            front_dist = self.get_min_distance_in_sector(-30, 30)
            if front_dist < self.get_parameter('obstacle_distance').value:
                self.update_cmd_vel(0.0, self.get_parameter('velocity.angular').value)
            else:
                self.update_cmd_vel(self.get_parameter('velocity.linear').value * 0.2, 0.0)
            return
        
        linear_speed = self.get_parameter('velocity.linear').value
        angular_speed = self.get_parameter('velocity.angular').value
        current_time = self.get_clock().now()
        
        # Check distances for obstacle avoidance
        front_dist = self.get_min_distance_in_sector(-30, 30)
        left_dist = self.get_min_distance_in_sector(30, 90)
        right_dist = self.get_min_distance_in_sector(-90, -30)
        
        # Obstacle avoidance takes priority
        is_stuck = (front_dist < self.get_parameter('obstacle_distance').value and 
                   abs(front_dist - self.last_front_distance) < 0.05 and
                   front_dist < 0.45)
        
        # Handle reverse behavior if stuck
        if (front_dist < self.get_parameter('obstacle_distance').value * 0.9 or is_stuck) and not self.is_reversing:
            self.is_reversing = True
            self.reverse_start_time = current_time
            if left_dist > right_dist:
                self.reverse_direction = 'LEFT'
            else:
                self.reverse_direction = 'RIGHT'
        
        if self.is_reversing:
            if self.reverse_start_time is not None:
                elapsed = (current_time - self.reverse_start_time).nanoseconds / 1e9
                if elapsed < self.reverse_duration:
                    reverse_speed = linear_speed * 0.5
                    if self.reverse_direction == 'LEFT':
                        self.update_cmd_vel(-reverse_speed, angular_speed * 0.6)
                    else:
                        self.update_cmd_vel(-reverse_speed, -angular_speed * 0.6)
                    return
                else:
                    self.is_reversing = False
                    self.reverse_start_time = None
        
        self.last_front_distance = front_dist
        
        # Obstacle avoidance priority
        if front_dist < self.get_parameter('obstacle_distance').value:
            # Obstacle ahead - turn away
            if left_dist > right_dist + 0.1:
                self.update_cmd_vel(0.0, angular_speed)
            elif right_dist > left_dist + 0.1:
                self.update_cmd_vel(0.0, -angular_speed)
            else:
                if left_dist > right_dist:
                    self.update_cmd_vel(0.0, angular_speed * 1.2)
                else:
                    self.update_cmd_vel(0.0, -angular_speed * 1.2)
        elif front_dist < self.get_parameter('safe_distance').value:
            # Approaching obstacle - slow down
            if left_dist > right_dist + 0.1:
                self.update_cmd_vel(linear_speed * 0.4, angular_speed * 0.6)
            elif right_dist > left_dist + 0.1:
                self.update_cmd_vel(linear_speed * 0.4, -angular_speed * 0.6)
            else:
                self.update_cmd_vel(linear_speed * 0.3, 0.0)
        else:
            # Clear path - navigate to frontier goal
            if self.current_goal is not None:
                angle_to_goal = self.calculate_angle_to_goal()
                
                if angle_to_goal is not None:
                    # Turn toward goal if not aligned
                    if abs(angle_to_goal) > 0.2:  # ~11 degrees
                        # Turn toward goal
                        self.update_cmd_vel(0.0, angular_speed * math.copysign(1.0, angle_to_goal))
                    else:
                        # Aligned - move toward goal
                        self.update_cmd_vel(linear_speed, angular_speed * angle_to_goal * 0.5)
                else:
                    # No valid angle - move forward slowly
                    self.update_cmd_vel(linear_speed * 0.3, 0.0)
            else:
                # No goal - use simple exploration
                if left_dist < self.get_parameter('obstacle_distance').value * 1.2:
                    self.update_cmd_vel(linear_speed, -angular_speed * 0.3)
                elif right_dist < self.get_parameter('obstacle_distance').value * 1.2:
                    self.update_cmd_vel(linear_speed, angular_speed * 0.3)
                else:
                    self.update_cmd_vel(linear_speed, 0.0)
        
        # Mark current position as visited periodically
        if self.robot_pose is not None:
            rx, ry, _ = self.robot_pose
            visited_key = (int(rx / self.cell_size), int(ry / self.cell_size))
            self.visited_cells.add(visited_key)


def main(args=None):
    rclpy.init(args=args)
    
    explorer = None
    try:
        explorer = FrontierExploreNode()
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

