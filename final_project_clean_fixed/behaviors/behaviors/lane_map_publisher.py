#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from visualization_msgs.msg import Marker


class LaneMapPublisher(Node):
    def __init__(self):
        super().__init__('lane_map_publisher')
        self.marker_pub = self.create_publisher(Marker, '/lane/map_markers', 10)
        self.create_subscription(Point, '/lane/error', self.lane_callback, 10)
        self.marker_id = 0
        self.get_logger().info('Lane map publisher started')

    def lane_callback(self, msg):
        if msg.y < 0.5:
            return

        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'recognized_lanes'
        marker.id = self.marker_id
        self.marker_id += 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Approximate lane point in front/right of robot for RViz display.
        marker.pose.position.x = 0.35
        marker.pose.position.y = -0.18
        marker.pose.position.z = 0.02
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.06
        marker.scale.y = 0.06
        marker.scale.z = 0.03

        marker.color.a = 0.8
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        marker.lifetime.sec = 30
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = LaneMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
