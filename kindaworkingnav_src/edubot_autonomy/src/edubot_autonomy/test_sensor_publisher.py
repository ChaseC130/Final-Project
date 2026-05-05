import math
import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
from rclpy.node import Node


class TestSensorPublisher(Node):
    def __init__(self):
        super().__init__('test_sensor_publisher')
        self.bridge = CvBridge()
        
        self.odom_pub = self.create_publisher(Odometry, '/wheel/odometry', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.front_camera_pub = self.create_publisher(Image, '/camera_1/image_raw', 1)
        self.down_camera_pub = self.create_publisher(Image, '/camera_2/image_raw', 1)
        
        self.time_step = 0
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.yaw = 0.0
        
        self.timer = self.create_timer(0.1, self.publish_sensors)
        self.get_logger().info('Test sensor publisher started')

    def publish_sensors(self):
        timestamp = self.get_clock().now().to_msg()
        
        # Publish odometry
        self.publish_odom(timestamp)
        
        # Publish scan
        self.publish_scan(timestamp)
        
        # Publish camera images
        if self.time_step % 10 == 0:
            self.publish_cameras(timestamp)
        
        self.time_step += 1

    def publish_odom(self, timestamp):
        msg = Odometry()
        msg.header = Header(stamp=timestamp, frame_id='odom')
        msg.child_frame_id = 'base_link'
        
        # Simulate position
        self.x_pos += 0.01 * math.cos(self.yaw)
        self.y_pos += 0.01 * math.sin(self.yaw)
        self.yaw += 0.001
        
        msg.pose.pose.position.x = self.x_pos
        msg.pose.pose.position.y = self.y_pos
        msg.pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        q = self.euler_to_quaternion(0, 0, self.yaw)
        msg.pose.pose.orientation = q
        
        msg.twist.twist.linear.x = 0.1
        msg.twist.twist.angular.z = 0.001
        
        self.odom_pub.publish(msg)

    def publish_scan(self, timestamp):
        msg = LaserScan()
        msg.header = Header(stamp=timestamp, frame_id='laser')
        msg.angle_min = -math.pi / 2
        msg.angle_max = math.pi / 2
        msg.angle_increment = math.pi / 180
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.1
        msg.range_max = 5.0
        
        ranges = []
        for i in range(181):
            angle = msg.angle_min + i * msg.angle_increment
            # Simulate simple box environment
            if -0.3 < angle < 0.3:
                ranges.append(0.5)  # Wall ahead
            elif angle < -1.0 or angle > 1.0:
                ranges.append(3.0)  # Far wall
            else:
                ranges.append(2.0)  # Side walls
        
        msg.ranges = ranges
        msg.intensities = [1.0] * len(ranges)
        
        self.scan_pub.publish(msg)

    def publish_cameras(self, timestamp):
        # Front camera: yellow lane line on black background
        front_img = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.line(front_img, (300, 400), (320, 100), (0, 255, 255), 5)  # Yellow line
        front_msg = self.bridge.cv2_to_imgmsg(front_img, 'bgr8')
        front_msg.header = Header(stamp=timestamp, frame_id='camera_1')
        self.front_camera_pub.publish(front_msg)
        
        # Downward camera: white lane marking on gray background
        down_img = np.full((480, 640, 3), 128, dtype=np.uint8)
        cv2.line(down_img, (250, 0), (250, 480), (255, 255, 255), 3)  # White right lane
        cv2.line(down_img, (300, 0), (300, 480), (200, 200, 200), 2)  # Dashed center
        down_msg = self.bridge.cv2_to_imgmsg(down_img, 'bgr8')
        down_msg.header = Header(stamp=timestamp, frame_id='camera_2')
        self.down_camera_pub.publish(down_msg)

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        
        return q


def main(args=None):
    rclpy.init(args=args)
    node = TestSensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
