#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from behaviors.behaviors import behavioral_coordination_pilot
from bbr_msgs.msg import VisualTarget
from tf2_ros import TransformListener, Buffer
from tf2_ros.transform_listener import TransformListener

# Global configuration for pose source
ODOM_TOPIC = '/odom'  # Change to use different odom topic
USE_TF2 = True  # Set to False to use /odom topic exclusively
TF2_BASE_FRAME = 'map'
TF2_ROBOT_FRAME = 'base_link'

def quaternion_to_yaw(qx, qy, qz, qw):
    # Compute yaw (z rotation) from quaternion
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return np.arctan2(siny_cosp, cosy_cosp)


class behavior_based_control(Node):
    def __init__(self):
        super().__init__('behavior_based_control')

        self.current_xyT=np.zeros((3),dtype=float)
        self.using_tf2 = USE_TF2

        # Initialize tf2 if enabled
        if self.using_tf2:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.get_logger().info("TF2 enabled for pose estimation")
            self.odom_sub = None
        else:
            self.get_logger().info(f"TF2 disabled, using {ODOM_TOPIC} for pose estimation")
            self.odom_sub = self.create_subscription(Odometry, ODOM_TOPIC, self.odom_cb, 10)

        # Subscriptions and publisher
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_cb, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/target_xy', self.goal_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.current_visual_target=None
        self.goal_pose = None  # store the latest goal pose

    def odom_cb(self, odom_msg: Odometry):
        # Only called when TF2 is disabled
        self.current_xyT[0] = odom_msg.pose.pose.position.x
        self.current_xyT[1] = odom_msg.pose.pose.position.y
        qx = odom_msg.pose.pose.orientation.x
        qy = odom_msg.pose.pose.orientation.y
        qz = odom_msg.pose.pose.orientation.z
        qw = odom_msg.pose.pose.orientation.w
        yaw = quaternion_to_yaw(qx, qy, qz, qw)
        self.current_xyT[2] = yaw

    def goal_cb(self, goal_msg: PoseStamped):
        """Store incoming goal pose from planner"""
        self.get_logger().info("Goal pose received in pilot")
        self.goal_pose = np.array([goal_msg.pose.position.x, goal_msg.pose.position.y])


    def get_pose_from_tf2(self):
        """
        Attempt to get pose from tf2 transform.
        Returns: (x, y, yaw) tuple on success, None on failure
        """
        try:
            transform = self.tf_buffer.lookup_transform(TF2_BASE_FRAME, TF2_ROBOT_FRAME, 
                                                        rclpy.time.Time())
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            yaw = quaternion_to_yaw(qx, qy, qz, qw)
            return (x, y, yaw)
        except Exception as e:
            self.get_logger().warn(f"TF2 lookup failed: {e}")
            return None

    def send_motion_cmd(self, xy_vector: np.ndarray):
        dist = np.sqrt(np.power(xy_vector, 2).sum())
        angle = np.arctan2(xy_vector[1], xy_vector[0])

        t_msg = Twist()
        t_msg.linear.x = 0.2 * dist
        t_msg.angular.z = float(angle)

        if np.abs(angle) > np.pi / 2.0:
            t_msg.linear.x = 0.05

        t_msg.linear.x = min(t_msg.linear.x, 1.0)
        t_msg.angular.z = min(t_msg.angular.z, 2.0)

        self.cmd_pub.publish(t_msg)

    def laser_cb(self, laser_msg: LaserScan):
        if self.goal_pose is None:
            print("No goal pose received...")
            return
        
        # Update pose (tf2 first if enabled, fallback to odom)
        if self.using_tf2:
            pose = self.get_pose_from_tf2()
            if pose is not None:
                self.current_xyT[0], self.current_xyT[1], self.current_xyT[2] = pose

        angles = np.array([
            laser_msg.angle_min + idx * laser_msg.angle_increment
            for idx in range(len(laser_msg.ranges))
        ])

        dist = np.array(laser_msg.ranges)
        valid_idx = np.where(dist > 0.02)[0]
        obs_xy = np.vstack((dist * np.cos(angles), dist * np.sin(angles))).transpose()
        obs_xy = obs_xy[valid_idx, :]

        motion_xy = behavioral_coordination_pilot(obs_xy, self.goal_pose, self.current_xyT)
        print(motion_xy)

        # Publish motion
        self.send_motion_cmd(motion_xy)


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
