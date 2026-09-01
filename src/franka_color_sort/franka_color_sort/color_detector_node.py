#!/usr/bin/env python3
"""
color_detector_node.py

ROS2 node that subscribes to a camera image topic, detects colored objects
(red, green, blue by default) using HSV thresholding, and publishes:
  - detected objects (color label + pixel centroid + bbox + area + world x/y)
    as JSON on a std_msgs/String topic
  - a debug image with bounding boxes/centroids drawn, for viewing in
    rviz2 or rqt_image_view

Pixel -> world (x, y) projection is a calibrated linear mapping for the
fixed overhead camera looking straight down at the z=0.42 object plane.
Calibrated from three known cube positions (red/green/blue) rather than
derived analytically from camera pose, since it matched ground truth
almost exactly and avoids needing precise intrinsics/extrinsics math.
"""

import json

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class ColorDetectorNode(Node):
    def __init__(self):
        super().__init__('color_detector_node')

        # ---- Parameters (override via ros2 param or launch file) ----
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('debug_image_topic', '/color_detector/debug_image')
        self.declare_parameter('detections_topic', '/color_detector/detections')
        self.declare_parameter('min_contour_area', 300)

        # Pixel->world calibration (fit from known cube positions).
        # world_x = origin_x - (pixel_y - center_y) / scale_x
        # world_y = origin_y - (pixel_x - center_x) / scale_y
        self.declare_parameter('center_x_px', 320.0)
        self.declare_parameter('center_y_px', 240.0)
        self.declare_parameter('scale_x_px_per_m', 466.67)
        self.declare_parameter('scale_y_px_per_m', 470.0)
        self.declare_parameter('origin_x_m', 0.6)
        self.declare_parameter('origin_y_m', 0.0)

        image_topic = self.get_parameter('image_topic').value
        debug_topic = self.get_parameter('debug_image_topic').value
        detections_topic = self.get_parameter('detections_topic').value
        self.min_contour_area = self.get_parameter('min_contour_area').value

        self.center_x_px = self.get_parameter('center_x_px').value
        self.center_y_px = self.get_parameter('center_y_px').value
        self.scale_x_px_per_m = self.get_parameter('scale_x_px_per_m').value
        self.scale_y_px_per_m = self.get_parameter('scale_y_px_per_m').value
        self.origin_x_m = self.get_parameter('origin_x_m').value
        self.origin_y_m = self.get_parameter('origin_y_m').value

        # ---- HSV color ranges ----
        self.color_ranges = {
            'red': [
                (np.array([0, 120, 70]), np.array([10, 255, 255])),
                (np.array([170, 120, 70]), np.array([180, 255, 255])),
            ],
            'green': [
                (np.array([36, 80, 70]), np.array([85, 255, 255])),
            ],
            'blue': [
                (np.array([94, 80, 70]), np.array([126, 255, 255])),
            ],
        }
        self.draw_colors_bgr = {
            'red': (0, 0, 255),
            'green': (0, 255, 0),
            'blue': (255, 0, 0),
        }

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)
        self.debug_pub = self.create_publisher(Image, debug_topic, 10)
        self.detections_pub = self.create_publisher(String, detections_topic, 10)

        self.get_logger().info(
            f"Color detector listening on '{image_topic}', "
            f"publishing detections on '{detections_topic}' "
            f"and debug image on '{debug_topic}'"
        )

    def pixel_to_world(self, px, py):
        world_x = self.origin_x_m - (py - self.center_y_px) / self.scale_x_px_per_m
        world_y = self.origin_y_m - (px - self.center_x_px) / self.scale_y_px_per_m
        return world_x, world_y

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'cv_bridge conversion failed: {exc}')
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        debug_frame = frame.copy()
        detections = []

        for color_name, ranges in self.color_ranges.items():
            mask = None
            for lower, upper in ranges:
                m = cv2.inRange(hsv, lower, upper)
                mask = m if mask is None else cv2.bitwise_or(mask, m)

            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for c in contours:
                area = cv2.contourArea(c)
                if area < self.min_contour_area:
                    continue

                x, y, w, h = cv2.boundingRect(c)
                cx, cy = x + w // 2, y + h // 2
                world_x, world_y = self.pixel_to_world(cx, cy)

                detections.append({
                    'color': color_name,
                    'pixel_x': cx,
                    'pixel_y': cy,
                    'world_x': round(world_x, 4),
                    'world_y': round(world_y, 4),
                    'bbox': [x, y, w, h],
                    'area': area,
                })

                draw_color = self.draw_colors_bgr[color_name]
                cv2.rectangle(debug_frame, (x, y), (x + w, y + h), draw_color, 2)
                cv2.circle(debug_frame, (cx, cy), 4, draw_color, -1)
                cv2.putText(
                    debug_frame, f"{color_name} ({world_x:.2f},{world_y:.2f})",
                    (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, draw_color, 2,
                )

        detections_msg = String()
        detections_msg.data = json.dumps({'detections': detections})
        self.detections_pub.publish(detections_msg)

        debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
