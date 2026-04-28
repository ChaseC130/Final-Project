#!/usr/bin/env python3

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from visualization_msgs.msg import Marker


class ConeDetector(Node):
    def __init__(self):
        super().__init__('cone_detector')

        self.declare_parameter('image_topic', '/camera_1/image_raw')
        self.declare_parameter('scan_topic', '/scan')
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.latest_scan = None

        self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.marker_pub = self.create_publisher(Marker, '/cones/markers', 10)

        self.marker_id = 0
        self.get_logger().info(f'Cone detector listening on image={image_topic}, scan={scan_topic}')

    def scan_callback(self, msg):
        self.latest_scan = msg

    def estimate_distance(self):
        if self.latest_scan is None or not self.latest_scan.ranges:
            return 1.0
        n = len(self.latest_scan.ranges)
        front = self.latest_scan.ranges[:max(1, n // 18)] + self.latest_scan.ranges[-max(1, n // 18):]
        good = [r for r in front if math.isfinite(r) and self.latest_scan.range_min < r < self.latest_scan.range_max]
        if not good:
            return 1.0
        return min(good)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f'Image conversion failed: {exc}')
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Orange cone threshold.
        mask = cv2.inRange(hsv, np.array([5, 100, 100]), np.array([25, 255, 255]))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return

        contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)
        if area < 300:
            return

        x, y, w, h = cv2.boundingRect(contour)
        cx = x + w / 2.0
        width = frame.shape[1]
        bearing = (cx - width / 2.0) / (width / 2.0) * 0.6
        dist = self.estimate_distance()

        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'traffic_cones'
        marker.id = self.marker_id
        self.marker_id += 1
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = float(dist * math.cos(bearing))
        marker.pose.position.y = float(dist * math.sin(bearing))
        marker.pose.position.z = 0.15
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.18
        marker.scale.y = 0.18
        marker.scale.z = 0.30

        marker.color.a = 0.9
        marker.color.r = 1.0
        marker.color.g = 0.35
        marker.color.b = 0.0
        marker.lifetime.sec = 5

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = ConeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
