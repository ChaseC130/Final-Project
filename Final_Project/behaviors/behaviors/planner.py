#!/usr/bin/env python3

from behaviors.grid import grid2D
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer

# Global configuration for pose source
ODOM_TOPIC = '/odom'
USE_TF2 = True
TF2_BASE_FRAME = 'map'
TF2_ROBOT_FRAME = 'base_link'


def quaternion_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return np.arctan2(siny_cosp, cosy_cosp)


class planner(Node):

    def __init__(self):
        super().__init__('planner')

        self.goal = None
        self.current_xyT = np.zeros((3), dtype=float)
        self.using_tf2 = USE_TF2

        # QoS for map subscription
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # TF2 setup
        if self.using_tf2:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.get_logger().info("TF2 enabled for pose estimation")
            self.odom_sub = None
        else:
            self.get_logger().info(f"TF2 disabled, using {ODOM_TOPIC}")
            self.odom_sub = self.create_subscription(
                Odometry, ODOM_TOPIC, self.odom_cb, 10
            )

        # Subscriptions
        self.map = None
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_cb, qos_profile=map_qos
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_cb, 10
        )
        self.timer_sub = self.create_timer(5.0, self.timer)

        # Publishers
        self.path_pub = self.create_publisher(Path, '/path', 10)
        self.target_pub = self.create_publisher(PoseStamped, '/target_xy', 10)

    # Map callback
    def map_cb(self, map_msg: OccupancyGrid):
        if self.map is None:
            self.get_logger().info("Map Received")
            self.map = grid2D(map_msg)

    # Pose retrieval
    def get_pose_from_tf2(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                TF2_BASE_FRAME, TF2_ROBOT_FRAME, rclpy.time.Time()
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y
            q = transform.transform.rotation
            yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

            return (x, y, yaw)

        except Exception as e:
            self.get_logger().warn(f"TF2 lookup failed: {e}")
            return None

    def odom_cb(self, odom_msg: Odometry):
        self.current_xyT[0] = odom_msg.pose.pose.position.x
        self.current_xyT[1] = odom_msg.pose.pose.position.y
        self.current_xyT[2] = quaternion_to_yaw(
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w
        )

    # Goal callback
    def goal_cb(self, target: PoseStamped):
        print("Goal Received")
        self.goal = np.array([target.pose.position.x, target.pose.position.y])


    # Path publishing
    def publish_path(self, path_arr: np.ndarray):
        pth = Path()
        pth.header.frame_id = "/map"
        pth.header.stamp = self.get_clock().now().to_msg()

        for x, y in path_arr:
            ps = PoseStamped()
            ps.header = pth.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            pth.poses.append(ps)

        self.path_pub.publish(pth)

    def publish_target(self, X, Y):
        ps = PoseStamped()
        ps.header.frame_id = "/map"
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = X
        ps.pose.position.y = Y
        self.target_pub.publish(ps)

    # Timer callback
    def timer(self):
        if self.goal is None:
            return

        # Update pose
        if self.using_tf2:
            pose = self.get_pose_from_tf2()
            if pose is None:
                self.get_logger().warn("TF2 failure, skipping update")
                return
            self.current_xyT[:] = pose

        if self.map is None:
            return

        start_rc = self.map.xy_to_rc(self.current_xyT[0], self.current_xyT[1])
        goal_rc = self.map.xy_to_rc(self.goal[0], self.goal[1])

        rc_path = self.astar(start_rc, goal_rc)
        if rc_path is None:
            self.get_logger().warn("No path found")
            return

        # Convert RC → XY
        path_xy = [self.map.rc_to_xy(r, c) for r, c in rc_path]
        path = np.array(path_xy)

        if len(path) > 0:
            self.publish_path(path)

            lookahead = min(10, len(path) - 1)
            target = path[lookahead]
            self.publish_target(target[0], target[1])

    # A* Search
    def astar(self, start_rc, goal_rc):
        import heapq

        open_set = []
        heapq.heappush(open_set, (0, start_rc))

        came_from = {}
        g_score = {start_rc: 0}

        def heuristic(a, b):
            return np.linalg.norm(np.array(a) - np.array(b))

        neighbors = [
            (-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1)
        ]

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal_rc:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start_rc)
                path.reverse()
                return path

            for dr, dc in neighbors:
                neighbor = (current[0] + dr, current[1] + dc)

                if not self.map.is_rc_in_bounds(neighbor[0], neighbor[1]):
                    continue

                val = self.map.get_value_rc(neighbor[0], neighbor[1])
                if val > 50 or val < 0:
                    continue

                # Prevent diagonal corner cutting
                if abs(dr) + abs(dc) == 2:
                    if self.map.get_value_rc(current[0]+dr, current[1]) > 50 or \
                       self.map.get_value_rc(current[0], current[1]+dc) > 50:
                        continue

                tentative_g = g_score[current] + np.linalg.norm([dr, dc])

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + heuristic(neighbor, goal_rc)
                    heapq.heappush(open_set, (f, neighbor))

        return None


# Main
def main(args=None):
    rclpy.init(args=args)
    node = planner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

