#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from behaviors.behaviors import behavioral_coordination


def quaternion_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return np.arctan2(siny_cosp, cosy_cosp)


def lidar_to_obstacle_xy(scan_msg):
    ranges = np.array(scan_msg.ranges, dtype=float)
    angles = scan_msg.angle_min + np.arange(len(ranges)) * scan_msg.angle_increment

    valid = np.isfinite(ranges)
    valid = np.logical_and(valid, ranges > scan_msg.range_min)
    valid = np.logical_and(valid, ranges < scan_msg.range_max)

    ranges = ranges[valid]
    angles = angles[valid]

    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)

    return np.column_stack((x, y))


def vector_to_twist(vec: np.ndarray, max_linear=0.2, max_angular=1.8):
    cmd = Twist()

    if vec is None or np.linalg.norm(vec) < 1e-6:
        return cmd

    desired_heading = np.arctan2(vec[1], vec[0])
    cmd.angular.z = np.clip(2.0 * desired_heading, -max_angular, max_angular)

    forward_scale = max(0.0, np.cos(desired_heading))
    cmd.linear.x = np.clip(max_linear * np.linalg.norm(vec) * forward_scale, 0.0, max_linear)

    return cmd


class behavior_based_control(Node):
    def __init__(self):
        super().__init__('behavior_based_control')

        self.current_xyT = np.zeros((3), dtype=float)
        self.current_target_xy = None
        self.current_obstacles_xy = np.zeros((0, 2), dtype=float)

        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.target_sub = self.create_subscription(PoseStamped, '/target_xy', self.target_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_cb)

    def target_cb(self, target_msg: PoseStamped):
        self.current_target_xy = np.array(
            [target_msg.pose.position.x, target_msg.pose.position.y],
            dtype=float
        )

    def odom_cb(self, odom_msg: Odometry):
        self.current_xyT[0] = odom_msg.pose.pose.position.x
        self.current_xyT[1] = odom_msg.pose.pose.position.y
        qx = odom_msg.pose.pose.orientation.x
        qy = odom_msg.pose.pose.orientation.y
        qz = odom_msg.pose.pose.orientation.z
        qw = odom_msg.pose.pose.orientation.w
        self.current_xyT[2] = quaternion_to_yaw(qx, qy, qz, qw)

    def laser_cb(self, laser_msg: LaserScan):
        self.current_obstacles_xy = lidar_to_obstacle_xy(laser_msg)

    def control_cb(self):
        if self.current_target_xy is None:
            return

        motion_xy = behavioral_coordination(
            visual_target=None,
            obstacle_xy=self.current_obstacles_xy,
            option='all',
            goal_xy=self.current_target_xy,
            current_xyT=self.current_xyT
        )

        cmd = vector_to_twist(motion_xy)
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = behavior_based_control()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
