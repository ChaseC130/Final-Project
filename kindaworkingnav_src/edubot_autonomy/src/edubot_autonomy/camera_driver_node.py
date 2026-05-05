import cv2
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.node import Node
import threading


class CameraDriverNode(Node):
    def __init__(self):
        super().__init__('camera_driver_node')

        self.bridge = CvBridge()

        self.front_camera_device = self.declare_parameter('front_camera_device', '/dev/video0').value
        self.down_camera_device = self.declare_parameter('down_camera_device', '/dev/video1').value
        self.frame_width = self.declare_parameter('frame_width', 640).value
        self.frame_height = self.declare_parameter('frame_height', 480).value
        self.fps = self.declare_parameter('fps', 30).value

        self.front_pub = self.create_publisher(Image, '/camera/image_raw', 1)
        self.down_pub = self.create_publisher(Image, '/camera_down/image_raw', 1)

        self.front_cap = None
        self.down_cap = None
        self.down_shared = False
        self.running = True

        self.front_cap = self._open_camera(self.front_camera_device, 'Front')
        self.down_cap = self._open_camera(self.down_camera_device, 'Downward')

        if self.front_cap is None and self.down_cap is not None:
            self.get_logger().warn('Front camera failed to open, reusing downward camera stream for front feed')
            self.front_cap = self.down_cap
            self.down_shared = True

        if self.down_cap is None and self.front_cap is not None:
            self.get_logger().warn('Downward camera failed to open, reusing front camera stream for downward feed')
            self.down_cap = self.front_cap
            self.down_shared = True

        self.timer = self.create_timer(1.0 / self.fps, self.capture_and_publish)
        self.get_logger().info('Camera driver node started')

    def capture_and_publish(self):
        front_frame = None
        if self.front_cap is not None and self.front_cap.isOpened():
            ret, frame = self.front_cap.read()
            if ret and frame is not None:
                try:
                    msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    self.front_pub.publish(msg)
                    front_frame = frame
                except Exception as e:
                    self.get_logger().warning(f'Front camera publish error: {e}')
            else:
                self.get_logger().warning('Front camera: failed to read frame')

        if self.down_cap is not None and self.down_cap.isOpened():
            if self.down_shared and front_frame is not None:
                frame = front_frame
            else:
                ret, frame = self.down_cap.read()
                if not (ret and frame is not None):
                    self.get_logger().warning('Downward camera: failed to read frame')
                    return
            try:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.down_pub.publish(msg)
            except Exception as e:
                self.get_logger().warning(f'Downward camera publish error: {e}')

    def __del__(self):
        self.running = False
        if self.front_cap is not None:
            self.front_cap.release()
        if self.down_cap is not None and self.down_cap is not self.front_cap:
            self.down_cap.release()

    def _open_camera(self, device, label):
        cap = None
        try:
            cap = cv2.VideoCapture(device)
            if cap is None or not cap.isOpened():
                self.get_logger().error(f'Failed to open {label.lower()} camera: {device}')
                return None
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            cap.set(cv2.CAP_PROP_FPS, self.fps)
            ret, frame = cap.read()
            if not ret or frame is None:
                self.get_logger().warning(f'{label} camera opened but failed to read frame: {device}')
                cap.release()
                return None
            self.get_logger().info(f'{label} camera opened: {device}')
            return cap
        except Exception as e:
            if cap is not None:
                cap.release()
            self.get_logger().error(f'{label} camera error: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = CameraDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
