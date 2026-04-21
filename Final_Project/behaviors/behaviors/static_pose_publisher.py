#!/usr/bin/env python3

"""Static pose publisher with odom repetition.

This node subscribes to ``/initialpose`` (PoseWithCovarianceStamped) and
updates an internal odometry message along with a ``map->odom`` transform.  A
timer republishes the odometry message at 10 Hz and constantly broadcasts the
transform so that late-joining components still see the correct frame.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import numpy as np
import pdb

def quaternion_to_yaw(qx, qy, qz, qw):
    # Compute yaw (z rotation) from quaternion
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return np.arctan2(siny_cosp, cosy_cosp)


class StaticPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__('static_pose_publisher')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.init_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initialpose_cb,
            10,
        )

        self._last_odom: Odometry | None = None
        self._map_to_odom = (0.0, 0.0, 0.0)

        self.create_timer(0.1, self._timer_cb)
        self.get_logger().info('static_pose_publisher running, waiting for /initialpose')

    def initialpose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose

        self.get_logger().info(
            f'initialpose received x={pose.position.x:.3f} y={pose.position.y:.3f}\u00b0'
        )
        # remember transform and odom
        # self._map_to_odom = (x, y, yaw)
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        # populate pose using the received coordinates
        odom.pose.pose.position.x = pose.position.x
        odom.pose.pose.position.y = pose.position.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = pose.orientation.x
        odom.pose.pose.orientation.y = pose.orientation.y
        odom.pose.pose.orientation.z = pose.orientation.z
        odom.pose.pose.orientation.w = pose.orientation.w
        self._last_odom = odom
        self.odom_pub.publish(odom)

    def _timer_cb(self) -> None:
        if self._last_odom is not None:
            odom = self._last_odom
            odom.header.stamp = self.get_clock().now().to_msg()
            self.odom_pub.publish(odom)
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self._last_odom.pose.pose.position.x
            t.transform.translation.y = self._last_odom.pose.pose.position.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x=self._last_odom.pose.pose.orientation.x
            t.transform.rotation.y=self._last_odom.pose.pose.orientation.y
            t.transform.rotation.z=self._last_odom.pose.pose.orientation.z
            t.transform.rotation.w=self._last_odom.pose.pose.orientation.w
            self.tf_broadcaster.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StaticPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
