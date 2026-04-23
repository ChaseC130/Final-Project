#!/usr/bin/env python3
"""
Lane Following Node
===================
- WHITE tape  = solid line  → always keep on the RIGHT, primary steering reference
- YELLOW tape = dashed line → on the LEFT, used to confirm lane position

SETUP: Run `ros2 topic list` on the robot and update the two
topic constants below to match your robot.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

# ─── CONFIGURE THESE TO MATCH YOUR ROBOT ──────────────────────────────────────
CAMERA_TOPIC  = '/camera/image_raw'
CMD_VEL_TOPIC = '/cmd_vel'
# ──────────────────────────────────────────────────────────────────────────────

# Driving parameters
LINEAR_SPEED  = 0.15    # m/s forward speed (start slow, increase once stable)
MAX_ANGULAR   = 0.8     # rad/s max turning speed
KP            = 0.005   # proportional gain — lower if wiggling, raise if sluggish

# Target: white solid line should sit at this fraction from the left of the image.
# 0.75 = 75% across → line is on the right side of the robot view.
TARGET_X_FRACTION = 0.75

# ─── HSV COLOR THRESHOLDS ─────────────────────────────────────────────────────
# White tape (solid line) — high value, low saturation
WHITE_LOWER  = np.array([0,   0, 180])
WHITE_UPPER  = np.array([180, 50, 255])

# Yellow tape (dashed line) — adjust hue range if needed under your lighting
YELLOW_LOWER = np.array([15,  80,  80])
YELLOW_UPPER = np.array([35, 255, 255])
# ──────────────────────────────────────────────────────────────────────────────

MIN_CONTOUR_AREA = 200  # ignore tiny blobs


class LaneFollower(Node):

    def __init__(self):
        super().__init__('lane_follower')
        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, 10)
        self.pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        self.get_logger().info(
            f'Lane follower started.\n'
            f'  Camera  : {CAMERA_TOPIC}\n'
            f'  CmdVel  : {CMD_VEL_TOPIC}\n'
            f'  Speed   : {LINEAR_SPEED} m/s\n'
            f'  White = solid (steer ref) | Yellow = dashed (left boundary)'
        )

    # ── Utilities ──────────────────────────────────────────────────────────────

    def get_largest_contour(self, mask):
        """Return the largest contour in a mask, or None if none found."""
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid = [c for c in contours if cv2.contourArea(c) > MIN_CONTOUR_AREA]
        if not valid:
            return None
        return max(valid, key=cv2.contourArea)

    def contour_centroid_x(self, contour):
        """Return the x-coordinate of a contour's centroid."""
        M = cv2.moments(contour)
        if M['m00'] == 0:
            return None
        return int(M['m10'] / M['m00'])

    # ── Main callback ──────────────────────────────────────────────────────────

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]

        # Crop to bottom 40% — closest road section
        roi = frame[int(h * 0.6):h, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # ── Detect both line colors ───────────────────────────────────────────
        white_mask   = cv2.inRange(hsv, WHITE_LOWER,  WHITE_UPPER)
        yellow_mask  = cv2.inRange(hsv, YELLOW_LOWER, YELLOW_UPPER)

        white_contour  = self.get_largest_contour(white_mask)
        yellow_contour = self.get_largest_contour(yellow_mask)

        white_cx  = self.contour_centroid_x(white_contour)  if white_contour  else None
        yellow_cx = self.contour_centroid_x(yellow_contour) if yellow_contour else None

        twist = Twist()

        # ── Steering logic ────────────────────────────────────────────────────
        if white_cx is not None:
            # Primary: steer to keep white solid line at TARGET_X_FRACTION
            target_x = int(w * TARGET_X_FRACTION)
            error    = white_cx - target_x

            angular  = -KP * error
            angular  = max(-MAX_ANGULAR, min(MAX_ANGULAR, angular))

            twist.linear.x  = LINEAR_SPEED
            twist.angular.z = angular

            self.get_logger().info(
                f'[WHITE] cx={white_cx} target={target_x} '
                f'err={error:+d} ang={angular:.3f}'
                + (f' | [YELLOW] cx={yellow_cx}' if yellow_cx else ' | no yellow'),
                throttle_duration_sec=0.5
            )

        elif yellow_cx is not None:
            # Fallback: only yellow visible → white line lost, steer right to find it
            self.get_logger().warn(
                'White line lost — using yellow to recover (steering right)',
                throttle_duration_sec=1.0
            )
            twist.linear.x  = LINEAR_SPEED * 0.5
            twist.angular.z = -0.3   # gentle right turn to find white line

        else:
            # No lines visible — stop and rotate slowly to search
            self.get_logger().warn(
                'No lines detected — searching...',
                throttle_duration_sec=2.0
            )
            twist.linear.x  = 0.0
            twist.angular.z = 0.2   # rotate left to scan

        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
