#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String


class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')

        self.declare_parameter('image_topic', '/camera_1/image_raw')
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)

        self.lane_error_pub = self.create_publisher(Point, '/lane/error', 10)
        self.intersection_pub = self.create_publisher(Bool, '/lane/intersection_detected', 10)
        self.end_line_pub = self.create_publisher(Bool, '/lane/end_line_detected', 10)
        self.debug_pub = self.create_publisher(Image, '/lane/debug_image', 10)
        self.status_pub = self.create_publisher(String, '/lane/status', 10)

        self.get_logger().info(f'Lane detector listening on {image_topic}')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f'Could not convert image: {exc}')
            return

        height, width = frame.shape[:2]

        # Use lower part of image because camera is downward facing.
        roi_y = int(height * 0.45)
        roi = frame[roi_y:height, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # White/yellow tape threshold. Tune these in the classroom if needed.
        white_mask = cv2.inRange(hsv, np.array([0, 0, 160]), np.array([180, 80, 255]))
        yellow_mask = cv2.inRange(hsv, np.array([15, 50, 80]), np.array([40, 255, 255]))
        lane_mask = cv2.bitwise_or(white_mask, yellow_mask)

        # Orange end-line threshold.
        orange_mask = cv2.inRange(hsv, np.array([5, 80, 80]), np.array([25, 255, 255]))

        kernel = np.ones((5, 5), np.uint8)
        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_OPEN, kernel)
        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_CLOSE, kernel)
        orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN, kernel)

        # Divide image into left, center, right regions.
        left_mask = lane_mask[:, :width // 3]
        center_mask = lane_mask[:, width // 3:2 * width // 3]
        right_mask = lane_mask[:, 2 * width // 3:]

        left_count = cv2.countNonZero(left_mask)
        center_count = cv2.countNonZero(center_mask)
        right_count = cv2.countNonZero(right_mask)

        # Solid lane should be on the right, so track the strongest right-side line.
        moments = cv2.moments(right_mask)
        lane_error = Point()
        lane_error.z = 0.0

        if moments['m00'] > 1000:
            cx_right = int(moments['m10'] / moments['m00']) + (2 * width // 3)
            desired_x = int(width * 0.72)
            error = float(cx_right - desired_x)
            lane_error.x = error
            lane_error.y = 1.0
        else:
            # No reliable right lane found.
            lane_error.x = 0.0
            lane_error.y = 0.0

        intersection = Bool()
        # Intersection likely if all three regions have strong tape visibility.
        intersection.data = left_count > 2500 and center_count > 2500 and right_count > 2500

        end_line = Bool()
        # End of road is perpendicular orange line.
        orange_count = cv2.countNonZero(orange_mask)
        end_line.data = orange_count > 4000

        self.lane_error_pub.publish(lane_error)
        self.intersection_pub.publish(intersection)
        self.end_line_pub.publish(end_line)

        status = String()
        status.data = (
            f'lane_valid={lane_error.y > 0.5}, error={lane_error.x:.1f}, '
            f'intersection={intersection.data}, end_line={end_line.data}'
        )
        self.status_pub.publish(status)

        # Debug image.
        debug = roi.copy()
        cv2.line(debug, (int(width * 0.72), 0), (int(width * 0.72), debug.shape[0]), (255, 0, 0), 2)
        if lane_error.y > 0.5:
            cx = int(lane_error.x + int(width * 0.72))
            cv2.circle(debug, (cx, debug.shape[0] // 2), 8, (0, 0, 255), -1)
        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
