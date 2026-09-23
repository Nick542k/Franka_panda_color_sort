#!/usr/bin/env python3
"""
Obstacle Avoider for Ridgeback AMR
Uses LiDAR to detect and avoid obstacles
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        
        # Parameters
        self.declare_parameter('safe_distance', 0.5)
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.5)
        
        self.safe_distance = self.get_parameter('safe_distance').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        
        # Subscriptions and publishers
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # State
        self.front_clear = True
        self.left_clear = True
        self.right_clear = True
        self.min_distance = float('inf')
        
        self.get_logger().info(
            f'Obstacle Avoider initialized. '
            f'Safe distance: {self.safe_distance}m, '
            f'Speed: {self.linear_speed} m/s'
        )

    def lidar_callback(self, msg: LaserScan):
        """Process LiDAR scan and update obstacle map"""
        ranges = msg.ranges
        num_rays = len(ranges)
        
        # Divide scan into sectors
        front_sector = ranges[int(num_rays*7/8):] + ranges[:int(num_rays/8)]
        left_sector = ranges[int(num_rays/4):int(num_rays/2)]
        right_sector = ranges[int(num_rays/2):int(num_rays*3/4)]
        
        # Check for obstacles
        self.front_clear = min(front_sector) > self.safe_distance
        self.left_clear = min(left_sector) > self.safe_distance
        self.right_clear = min(right_sector) > self.safe_distance
        self.min_distance = min(ranges)
        
        # Publish velocity command based on obstacles
        self.decide_action()

    def decide_action(self):
        """Decide robot action based on obstacle positions"""
        msg = Twist()
        
        # Hierarchy: stop > turn > forward
        if not self.front_clear:
            # Obstacle ahead
            if self.left_clear and not self.right_clear:
                # Turn left
                msg.angular.z = self.angular_speed
                self.get_logger().info('Obstacle ahead -> turning left')
            elif self.right_clear and not self.left_clear:
                # Turn right
                msg.angular.z = -self.angular_speed
                self.get_logger().info('Obstacle ahead -> turning right')
            elif self.left_clear and self.right_clear:
                # Both sides clear, prefer left (45°)
                msg.linear.x = self.linear_speed * 0.5
                msg.angular.z = self.angular_speed
                self.get_logger().info('Obstacle ahead -> turning left (both clear)')
            else:
                # Blocked, stop
                self.get_logger().warn('BLOCKED! All directions blocked')
        else:
            # No front obstacle, move forward
            msg.linear.x = self.linear_speed
            
            # Adjust heading if sides are imbalanced
            if not self.left_clear and self.right_clear:
                msg.angular.z = -0.1  # Slight right turn
            elif not self.right_clear and self.left_clear:
                msg.angular.z = 0.1  # Slight left turn
        
        # Alert if very close
        if self.min_distance < 0.3:
            self.get_logger().error(f'CRITICAL: {self.min_distance:.2f}m')
            msg.linear.x = 0.0
            msg.angular.z = self.angular_speed  # Spin in place
        
        self.cmd_vel_pub.publish(msg)

    def run_demo(self):
        """Run avoidance demo"""
        self.get_logger().info('Running obstacle avoidance demo (Ctrl+C to stop)...')
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            self.get_logger().info('Demo stopped')
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    node.run_demo()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
