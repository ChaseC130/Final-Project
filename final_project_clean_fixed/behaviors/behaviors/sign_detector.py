#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class SignDetector(Node):
    def __init__(self):
        super().__init__('sign_detector')

        self.declare_parameter('image_topic', '/camera_1/image_raw')
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.sign_pub = self.create_publisher(String, '/sign/type', 10)

        self.get_logger().info(f'Sign detector listening on {image_topic}')

    def publish_sign(self, sign_name):
        msg = String()
        msg.data = sign_name
        self.sign_pub.publish(msg)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Simple color-based placeholder:
        # red large object = stop sign
        # orange/red broad sign = road closed candidate
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        red_mask = cv2.bitwise_or(
            cv2.inRange(hsv, lower_red1, upper_red1),
            cv2.inRange(hsv, lower_red2, upper_red2)
        )

        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return

        contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)
        if area < 1000:
            return

        x, y, w, h = cv2.boundingRect(contour)
        ratio = w / float(h)

        if 0.75 < ratio < 1.25:
            self.publish_sign('stop')
        elif ratio >= 1.25:
            self.publish_sign('road_closed')


def main(args=None):
    rclpy.init(args=args)
    node = SignDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
