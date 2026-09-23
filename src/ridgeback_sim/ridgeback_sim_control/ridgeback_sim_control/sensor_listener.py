#!/usr/bin/env python3
"""
Sensor Listener for Ridgeback AMR
Subscribes to odometry, IMU, and LiDAR topics
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
import math


class SensorListener(Node):
    def __init__(self):
        super().__init__('sensor_listener')
        
        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10
        )
        
        self.get_logger().info('Sensor Listener initialized')
        
        # State
        self.current_pose = None
        self.current_velocity = None
        self.min_distance = float('inf')
        
    def odom_callback(self, msg: Odometry):
        """Handle odometry updates"""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        # Convert quaternion to Euler angles (simplified)
        yaw = 2 * math.atan2(orientation.z, orientation.w)
        
        velocity = msg.twist.twist
        speed = math.sqrt(velocity.linear.x**2 + velocity.linear.y**2)
        
        self.current_pose = (position.x, position.y, yaw)
        self.current_velocity = speed
        
        if self.get_clock().now().nanoseconds % int(5e9) < 1e7:  # Log every ~5s
            self.get_logger().info(
                f'Pose: x={position.x:.2f}, y={position.y:.2f}, yaw={yaw:.2f} rad | '
                f'Speed: {speed:.2f} m/s'
            )
    
    def imu_callback(self, msg: Imu):
        """Handle IMU updates"""
        accel = msg.linear_acceleration
        accel_mag = math.sqrt(accel.x**2 + accel.y**2 + accel.z**2)
        
        angular_vel = msg.angular_velocity
        angular_mag = math.sqrt(angular_vel.x**2 + angular_vel.y**2 + angular_vel.z**2)
        
        if self.get_clock().now().nanoseconds % int(10e9) < 1e7:  # Log every ~10s
            self.get_logger().info(
                f'IMU - Accel mag: {accel_mag:.2f} m/s² | '
                f'Angular vel: {angular_mag:.2f} rad/s'
            )
    
    def lidar_callback(self, msg: LaserScan):
        """Handle LiDAR scans"""
        # Find minimum distance
        min_dist = float('inf')
        for distance in msg.ranges:
            if distance > msg.range_min and distance < msg.range_max:
                if distance < min_dist:
                    min_dist = distance
        
        self.min_distance = min_dist
        
        if self.get_logger().get_effective_level() == rclpy.logging.LoggingSeverity.DEBUG:
            num_rays = len(msg.ranges)
            self.get_logger().debug(
                f'LiDAR: {num_rays} rays, min distance: {min_dist:.2f}m, '
                f'range: [{msg.range_min:.1f}, {msg.range_max:.1f}]'
            )
        
        # Alert if obstacle nearby
        if min_dist < 1.0:
            self.get_logger().warn(f'OBSTACLE DETECTED: {min_dist:.2f}m')
    
    def print_status(self):
        """Print current robot status"""
        self.get_logger().info('=== Ridgeback Status ===')
        if self.current_pose:
            self.get_logger().info(
                f'Position: ({self.current_pose[0]:.2f}, {self.current_pose[1]:.2f}), '
                f'Heading: {self.current_pose[2]:.2f} rad'
            )
        if self.current_velocity is not None:
            self.get_logger().info(f'Current velocity: {self.current_velocity:.2f} m/s')
        if self.min_distance != float('inf'):
            self.get_logger().info(f'Nearest obstacle: {self.min_distance:.2f}m')


def main(args=None):
    rclpy.init(args=args)
    node = SensorListener()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.print_status()
        node.get_logger().info('Sensor Listener shutting down')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
