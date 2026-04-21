#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class SimpleExplorer(Node):
    def __init__(self):
        super().__init__('simple_explorer')

        self.scan_msg = None
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        self.turn_direction = 1.0
        self.turn_steps_remaining = 0

        self.timer = self.create_timer(0.1, self.control_cb)

    def scan_cb(self, msg: LaserScan):
        self.scan_msg = msg

    def get_sector_min(self, ranges, start_idx, end_idx):
        sector = np.array(ranges[start_idx:end_idx], dtype=float)
        sector = sector[np.isfinite(sector)]
        if sector.size == 0:
            return np.inf
        return np.min(sector)

    def control_cb(self):
        if self.scan_msg is None:
            return

        ranges = self.scan_msg.ranges
        n = len(ranges)
        if n == 0:
            return

        mid = n // 2
        width = max(10, n // 12)

        front_min = self.get_sector_min(ranges, max(0, mid - width), min(n, mid + width))
        left_min = self.get_sector_min(ranges, min(n - 1, mid + width), min(n, mid + 3 * width))
        right_min = self.get_sector_min(ranges, max(0, mid - 3 * width), max(1, mid - width))

        cmd = Twist()

        if self.turn_steps_remaining > 0:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.8 * self.turn_direction
            self.turn_steps_remaining -= 1
            self.cmd_pub.publish(cmd)
            return

        if front_min < 0.6:
            if left_min > right_min:
                self.turn_direction = 1.0
            elif right_min > left_min:
                self.turn_direction = -1.0
            else:
                self.turn_direction = 1.0 if np.random.rand() > 0.5 else -1.0

            self.turn_steps_remaining = np.random.randint(8, 18)
            cmd.linear.x = 0.0
            cmd.angular.z = 0.8 * self.turn_direction
        else:
            cmd.linear.x = 0.15

            steer = 0.0
            if left_min < 0.8:
                steer -= 0.3
            if right_min < 0.8:
                steer += 0.3

            steer += np.random.uniform(-0.08, 0.08)
            cmd.angular.z = steer

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        node.cmd_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
