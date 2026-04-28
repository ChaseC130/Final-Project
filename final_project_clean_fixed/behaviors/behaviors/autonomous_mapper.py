#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class AutonomousMapper(Node):
    def __init__(self):
        super().__init__('autonomous_mapper')
        self.get_logger().info(
            'AutonomousMapper placeholder running. SLAM is launched through slam_toolbox.'
        )


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
