#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


class RoadNavigator(Node):
    def __init__(self):
        super().__init__('road_navigator')

        self.declare_parameter('scan_topic', '/scan')
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(Point, '/lane/error', self.lane_callback, 10)
        self.create_subscription(Bool, '/lane/intersection_detected', self.intersection_callback, 10)
        self.create_subscription(Bool, '/lane/end_line_detected', self.end_line_callback, 10)
        self.create_subscription(String, '/sign/type', self.sign_callback, 10)
        self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)

        self.timer = self.create_timer(0.05, self.control_loop)

        self.lane_valid = False
        self.lane_error = 0.0
        self.intersection_detected = False
        self.end_line_detected = False
        self.sign_type = 'none'

        self.front_obstacle = False
        self.left_clear = True
        self.right_clear = True

        self.state = 'FOLLOW_LANE'
        self.state_start = time.time()

        self.linear_speed = 0.18
        self.turn_speed = 0.65
        self.kp = 0.004

        self.get_logger().info('Road navigator started')

    def set_state(self, state):
        if state != self.state:
            self.get_logger().info(f'State: {self.state} -> {state}')
            self.state = state
            self.state_start = time.time()

    def lane_callback(self, msg):
        self.lane_error = msg.x
        self.lane_valid = msg.y > 0.5

    def intersection_callback(self, msg):
        self.intersection_detected = msg.data

    def end_line_callback(self, msg):
        self.end_line_detected = msg.data

    def sign_callback(self, msg):
        self.sign_type = msg.data.lower().strip()

    def scan_callback(self, msg):
        if not msg.ranges:
            return

        ranges = list(msg.ranges)
        n = len(ranges)

        def clean(vals):
            good = [v for v in vals if math.isfinite(v) and msg.range_min < v < msg.range_max]
            if not good:
                return 99.0
            return min(good)

        front = ranges[:max(1, n // 12)] + ranges[-max(1, n // 12):]
        left = ranges[n // 6:n // 3]
        right = ranges[2 * n // 3:5 * n // 6]

        front_min = clean(front)
        left_min = clean(left)
        right_min = clean(right)

        self.front_obstacle = front_min < 0.45
        self.left_clear = left_min > 0.55
        self.right_clear = right_min > 0.55

    def control_loop(self):
        now = time.time()
        elapsed = now - self.state_start
        cmd = Twist()

        # Priority behaviors.
        if self.sign_type == 'stop':
            self.set_state('STOP_SIGN')
            self.sign_type = 'none'
        elif self.sign_type == 'road_closed':
            self.set_state('TURN_AROUND')
            self.sign_type = 'none'
        elif self.end_line_detected and self.state == 'FOLLOW_LANE':
            self.set_state('TURN_AROUND')
        elif self.intersection_detected and self.state == 'FOLLOW_LANE':
            if self.sign_type == 'one_way_wrong':
                self.set_state('TURN_AROUND')
            else:
                self.set_state('RIGHT_TURN')
        elif self.front_obstacle and self.state == 'FOLLOW_LANE':
            if self.left_clear:
                self.set_state('AVOID_LEFT')
            else:
                self.set_state('WAIT_FOR_OBSTACLE')

        if self.state == 'FOLLOW_LANE':
            if self.lane_valid:
                cmd.linear.x = self.linear_speed
                cmd.angular.z = -self.kp * self.lane_error
            else:
                # Slow search for the right lane line.
                cmd.linear.x = 0.04
                cmd.angular.z = -0.25

        elif self.state == 'RIGHT_TURN':
            # Timed right turn. Tune duration on robot.
            if elapsed < 1.6:
                cmd.linear.x = 0.05
                cmd.angular.z = -self.turn_speed
            else:
                self.set_state('FOLLOW_LANE')

        elif self.state == 'TURN_AROUND':
            # Timed 180-degree turn. Tune duration on robot.
            if elapsed < 3.1:
                cmd.linear.x = 0.0
                cmd.angular.z = self.turn_speed
            else:
                self.end_line_detected = False
                self.set_state('FOLLOW_LANE')

        elif self.state == 'AVOID_LEFT':
            # Move around obstacle by crossing dashed lane when possible.
            if elapsed < 1.0:
                cmd.linear.x = 0.08
                cmd.angular.z = 0.45
            elif elapsed < 2.1:
                cmd.linear.x = 0.15
                cmd.angular.z = -0.25
            else:
                self.set_state('FOLLOW_LANE')

        elif self.state == 'WAIT_FOR_OBSTACLE':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            if not self.front_obstacle:
                self.set_state('FOLLOW_LANE')

        elif self.state == 'STOP_SIGN':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            if elapsed > 5.0:
                self.set_state('FOLLOW_LANE')

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = RoadNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
