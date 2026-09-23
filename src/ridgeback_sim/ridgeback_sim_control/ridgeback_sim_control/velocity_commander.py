#!/usr/bin/env python3
"""
Velocity Commander for Ridgeback AMR
Publishes geometry_msgs/Twist to /cmd_vel topic
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys


class VelocityCommander(Node):
    def __init__(self):
        super().__init__('velocity_commander')
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Velocity Commander initialized. Publishing to /cmd_vel')
        
    def publish_velocity(self, linear_x, angular_z):
        """Publish velocity command to robot"""
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(angular_z)
        
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info(f'Published: linear_x={linear_x}, angular_z={angular_z}')

    def move_forward(self, speed=0.5, duration=5.0):
        """Move forward for specified duration"""
        self.get_logger().info(f'Moving forward at {speed} m/s for {duration} seconds')
        end_time = self.get_clock().now().nanoseconds + (int(duration * 1e9))
        
        while self.get_clock().now().nanoseconds < end_time:
            self.publish_velocity(speed, 0.0)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Stop
        self.publish_velocity(0.0, 0.0)
        self.get_logger().info('Stopped')

    def rotate(self, angular_velocity=0.5, duration=4.0):
        """Rotate in place for specified duration"""
        self.get_logger().info(f'Rotating at {angular_velocity} rad/s for {duration} seconds')
        end_time = self.get_clock().now().nanoseconds + (int(duration * 1e9))
        
        while self.get_clock().now().nanoseconds < end_time:
            self.publish_velocity(0.0, angular_velocity)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Stop
        self.publish_velocity(0.0, 0.0)
        self.get_logger().info('Stopped')

    def interactive_mode(self):
        """Interactive velocity control"""
        self.get_logger().info('Entering interactive mode. Commands: f/b/l/r/s (forward/back/left/right/stop)')
        
        try:
            while True:
                cmd = input('Command (f/b/l/r/s/q): ').strip().lower()
                
                if cmd == 'f':
                    self.publish_velocity(0.5, 0.0)
                elif cmd == 'b':
                    self.publish_velocity(-0.5, 0.0)
                elif cmd == 'l':
                    self.publish_velocity(0.0, 0.5)
                elif cmd == 'r':
                    self.publish_velocity(0.0, -0.5)
                elif cmd == 's':
                    self.publish_velocity(0.0, 0.0)
                elif cmd == 'q':
                    self.publish_velocity(0.0, 0.0)
                    break
                else:
                    self.get_logger().warn('Unknown command')
        except KeyboardInterrupt:
            self.publish_velocity(0.0, 0.0)
            self.get_logger().info('Interrupted')


def main(args=None):
    rclpy.init(args=args)
    node = VelocityCommander()
    
    try:
        if len(sys.argv) > 1:
            if sys.argv[1] == 'forward':
                node.move_forward(speed=0.5, duration=5.0)
            elif sys.argv[1] == 'rotate':
                node.rotate(angular_velocity=0.5, duration=4.0)
            elif sys.argv[1] == 'interactive':
                node.interactive_mode()
            else:
                node.get_logger().error('Unknown mode. Use: forward, rotate, interactive')
        else:
            node.get_logger().info('Usage: velocity_commander <mode>')
            node.get_logger().info('Modes: forward | rotate | interactive')
            node.interactive_mode()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
