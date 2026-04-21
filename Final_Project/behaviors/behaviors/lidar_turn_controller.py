#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math


class LidarTurnController(Node):
    def __init__(self):
        super().__init__('lidar_turn_controller')

        # Declare parameters
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('kp', 1.0)  # Proportional gain for turning

        self.cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        self.kp = self.get_parameter('kp').get_parameter_value().double_value

        # Subscription and publisher
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)

        self.get_logger().info(f'Lidar turn controller started, publishing to {self.cmd_topic}')

    def scan_callback(self, msg: LaserScan):
        # Find the index of the minimum range (closest point)
        ranges = msg.ranges
        if not ranges:
            return

        # Filter out inf and nan
        valid_ranges = []
        valid_indices = []
        for i, r in enumerate(ranges):
            if math.isfinite(r) and r > msg.range_min and r < msg.range_max:
                valid_ranges.append(r)
                valid_indices.append(i)

        if not valid_ranges:
            return

        min_index = valid_indices[valid_ranges.index(min(valid_ranges))]

        # Calculate angle of the closest point
        angle = msg.angle_min + min_index * msg.angle_increment

        # Publish twist to turn towards the angle
        twist = Twist()
        twist.angular.z = self.kp * angle  # Simple proportional control

        self.cmd_pub.publish(twist)
        print(f'Closest point at angle {angle:.2f}, publishing angular.z {twist.angular.z:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = LidarTurnController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()