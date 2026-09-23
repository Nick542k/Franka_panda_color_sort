#!/usr/bin/env python3
"""
Autonomous Navigator for Ridgeback AMR
Navigates to goal while avoiding obstacles
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math


class AutonomousNavigator(Node):
    def __init__(self):
        super().__init__('autonomous_navigator')
        
        # Parameters
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('obstacle_distance', 0.6)
        
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        
        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10
        )
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # State
        self.current_pose = None
        self.current_yaw = 0.0
        self.goal_pose = None
        self.min_distance = float('inf')
        self.front_clear = True
        
        self.get_logger().info('Autonomous Navigator initialized')
        self.get_logger().info('Call set_goal(x, y) to navigate')
    
    def odom_callback(self, msg: Odometry):
        """Update current pose"""
        self.current_pose = msg.pose.pose.position
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        self.current_yaw = 2 * math.atan2(q.z, q.w)
    
    def lidar_callback(self, msg: LaserScan):
        """Update obstacle info"""
        ranges = msg.ranges
        num_rays = len(ranges)
        
        # Check front sector
        front_sector = ranges[int(num_rays*7/8):] + ranges[:int(num_rays/8)]
        self.front_clear = min(front_sector) > self.obstacle_distance
        self.min_distance = min(ranges)
    
    def set_goal(self, goal_x, goal_y):
        """Set navigation goal"""
        self.goal_pose = (goal_x, goal_y)
        self.get_logger().info(f'Goal set to: ({goal_x}, {goal_y})')
    
    def navigate_to_goal(self):
        """Main navigation loop"""
        if not self.goal_pose or not self.current_pose:
            self.get_logger().warn('Goal or pose not set yet')
            return False
        
        # Calculate distance and angle to goal
        dx = self.goal_pose[0] - self.current_pose.x
        dy = self.goal_pose[1] - self.current_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        goal_angle = math.atan2(dy, dx)
        
        # Angle error (wrapped to [-pi, pi])
        angle_error = self.wrap_angle(goal_angle - self.current_yaw)
        
        # Check if goal reached
        if distance < self.goal_tolerance:
            self.get_logger().info(f'Goal reached!')
            return True
        
        # Create velocity command
        msg = Twist()
        
        # If facing goal, move forward; else turn first
        if abs(angle_error) > 0.3:  # ~17 degrees
            # Turn to face goal
            msg.angular.z = self.max_angular_speed * 0.5 if angle_error > 0 else -self.max_angular_speed * 0.5
            msg.linear.x = 0.0
            status = 'turning'
        else:
            # Move toward goal
            msg.linear.x = min(self.max_linear_speed, distance)
            msg.angular.z = angle_error * 0.5  # Proportional steering
            status = 'moving'
        
        # Emergency stop if obstacle too close
        if self.min_distance < 0.3:
            self.get_logger().error(f'EMERGENCY STOP: obstacle {self.min_distance:.2f}m away')
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            status = 'stopped'
        elif not self.front_clear:
            self.get_logger().warn('Obstacle ahead, stopping')
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            status = 'obstacle'
        
        self.cmd_vel_pub.publish(msg)
        
        self.get_logger().info(
            f'{status.upper()}: distance={distance:.2f}m, '
            f'angle_error={angle_error:.2f}rad, '
            f'linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}'
        )
        
        return False
    
    @staticmethod
    def wrap_angle(angle):
        """Wrap angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousNavigator()
    
    # Set goal and navigate
    node.set_goal(3.0, 3.0)
    
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
            goal_reached = node.navigate_to_goal()
            if goal_reached:
                break
    except KeyboardInterrupt:
        stop_msg = Twist()
        node.cmd_vel_pub.publish(stop_msg)
        node.get_logger().info('Navigation stopped')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
