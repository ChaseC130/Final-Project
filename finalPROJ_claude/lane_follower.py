#!/usr/bin/env python3
"""
Lane Following Node for Edubot
Keeps robot in lane, respecting solid white lines and yellow dashed lines
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class LaneFollower(Node):
    def __init__(self):
        super().__init__('lane_follower')
        
        # Parameters
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('linear_speed', 0.2)  # m/s
        self.declare_parameter('max_angular_speed', 0.5)  # rad/s
        
        # PID parameters for lane centering
        self.declare_parameter('kp', 0.005)  # Proportional gain
        self.declare_parameter('ki', 0.0001)  # Integral gain
        self.declare_parameter('kd', 0.001)  # Derivative gain
        
        # Lane detection parameters
        self.declare_parameter('roi_height', 0.3)  # Bottom 30% of image
        self.declare_parameter('min_line_length', 20)
        self.declare_parameter('max_line_gap', 10)
        
        # Get parameters
        camera_topic = self.get_parameter('camera_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        
        self.roi_height = self.get_parameter('roi_height').value
        self.min_line_length = self.get_parameter('min_line_length').value
        self.max_line_gap = self.get_parameter('max_line_gap').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # PID control variables
        self.error_sum = 0.0
        self.last_error = 0.0
        
        # Lane state
        self.solid_line_x = None  # x-position of solid white line (should be on right)
        self.dashed_line_x = None  # x-position of dashed yellow line (on left)
        self.lane_center = None
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.debug_image_pub = self.create_publisher(Image, '/lane_debug', 10)
        
        # Control timer (20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('Lane Follower Node Initialized')
    
    def detect_white_line(self, roi):
        """Detect solid white line using color filtering and Hough transform"""
        # Convert to HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # White color range (high value, low saturation)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 30, 255])
        
        # Create mask
        white_mask = cv2.inRange(hsv, lower_white, upper_white)
        
        # Apply morphological operations
        kernel = np.ones((3, 3), np.uint8)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
        
        # Detect edges
        edges = cv2.Canny(white_mask, 50, 150)
        
        # Hough line detection
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi/180,
            threshold=30,
            minLineLength=self.min_line_length,
            maxLineGap=self.max_line_gap
        )
        
        return lines, white_mask
    
    def detect_yellow_line(self, roi):
        """Detect dashed yellow line using color filtering"""
        # Convert to HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Yellow color range
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        
        # Create mask
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        # Apply morphological operations
        kernel = np.ones((3, 3), np.uint8)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
        
        # Detect edges
        edges = cv2.Canny(yellow_mask, 50, 150)
        
        # Hough line detection
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi/180,
            threshold=20,
            minLineLength=self.min_line_length,
            maxLineGap=self.max_line_gap
        )
        
        return lines, yellow_mask
    
    def get_line_position(self, lines, image_width):
        """
        Get average x-position of detected lines
        Returns the x-coordinate of the line in image coordinates
        """
        if lines is None or len(lines) == 0:
            return None
        
        x_positions = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            # Get center x position of the line
            x_positions.append((x1 + x2) / 2.0)
        
        if len(x_positions) == 0:
            return None
        
        # Return median x position (more robust than mean)
        return np.median(x_positions)
    
    def image_callback(self, msg):
        """Process camera image to detect lane lines"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width = cv_image.shape[:2]
            
            # Define ROI (region of interest) - bottom portion of image
            roi_start = int(height * (1 - self.roi_height))
            roi = cv_image[roi_start:, :]
            
            # Detect white solid line (should be on the right)
            white_lines, white_mask = self.detect_white_line(roi)
            self.solid_line_x = self.get_line_position(white_lines, width)
            
            # Detect yellow dashed line (on the left)
            yellow_lines, yellow_mask = self.detect_yellow_line(roi)
            self.dashed_line_x = self.get_line_position(yellow_lines, width)
            
            # Calculate lane center
            if self.solid_line_x is not None and self.dashed_line_x is not None:
                # Both lines detected - center between them
                self.lane_center = (self.solid_line_x + self.dashed_line_x) / 2.0
            elif self.solid_line_x is not None:
                # Only solid line detected - stay left of it
                # Assume lane width and calculate center
                assumed_lane_width = 200  # pixels (adjust based on your setup)
                self.lane_center = self.solid_line_x - assumed_lane_width / 2.0
            elif self.dashed_line_x is not None:
                # Only dashed line detected - stay right of it
                assumed_lane_width = 200
                self.lane_center = self.dashed_line_x + assumed_lane_width / 2.0
            else:
                # No lines detected
                self.lane_center = None
            
            # Create debug visualization
            debug_img = roi.copy()
            
            # Draw detected white lines in blue
            if white_lines is not None:
                for line in white_lines:
                    x1, y1, x2, y2 = line[0]
                    cv2.line(debug_img, (x1, y1), (x2, y2), (255, 0, 0), 2)
            
            # Draw detected yellow lines in green
            if yellow_lines is not None:
                for line in yellow_lines:
                    x1, y1, x2, y2 = line[0]
                    cv2.line(debug_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Draw lane center
            if self.lane_center is not None:
                cv2.line(debug_img, 
                        (int(self.lane_center), 0), 
                        (int(self.lane_center), roi.shape[0]), 
                        (0, 255, 255), 2)
            
            # Draw image center (robot's current position)
            image_center = width / 2
            cv2.line(debug_img, 
                    (int(image_center), 0), 
                    (int(image_center), roi.shape[0]), 
                    (0, 0, 255), 2)
            
            # Add text information
            if self.solid_line_x is not None:
                cv2.putText(debug_img, f"Solid: {int(self.solid_line_x)}", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            if self.dashed_line_x is not None:
                cv2.putText(debug_img, f"Dashed: {int(self.dashed_line_x)}", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            if self.lane_center is not None:
                cv2.putText(debug_img, f"Center: {int(self.lane_center)}", 
                           (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Publish debug image
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            self.debug_image_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def control_loop(self):
        """PID control loop to keep robot centered in lane"""
        twist = Twist()
        
        if self.lane_center is None:
            # No lane detected - stop
            self.get_logger().warn('No lane detected - stopping')
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            return
        
        # Get image center (where robot currently is)
        image_center = 320  # Typical camera width / 2 (adjust if needed)
        
        # Calculate error (positive = need to turn right, negative = need to turn left)
        error = self.lane_center - image_center
        
        # PID control
        self.error_sum += error
        error_diff = error - self.last_error
        
        # Calculate angular velocity
        angular_z = -(self.kp * error + self.ki * self.error_sum + self.kd * error_diff)
        
        # Clamp angular velocity
        angular_z = np.clip(angular_z, -self.max_angular_speed, self.max_angular_speed)
        
        # Safety check: if solid line is too close on the right, turn more left
        if self.solid_line_x is not None:
            distance_to_solid = self.solid_line_x - image_center
            min_distance = 50  # pixels (adjust based on robot width)
            
            if distance_to_solid < min_distance:
                # Too close to solid line! Turn left more aggressively
                self.get_logger().warn(f'Too close to solid line! Distance: {distance_to_solid:.1f}')
                angular_z = min(angular_z - 0.2, -0.1)  # Force left turn
        
        # Set velocities
        twist.linear.x = self.linear_speed
        twist.angular.z = angular_z
        
        # Publish command
        self.cmd_vel_pub.publish(twist)
        
        # Update PID state
        self.last_error = error
        
        # Log status
        if self.get_clock().now().nanoseconds % 1000000000 < 50000000:  # Log every ~1 second
            self.get_logger().info(
                f'Error: {error:.1f}, Angular: {angular_z:.3f}, '
                f'Solid: {self.solid_line_x}, Dashed: {self.dashed_line_x}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
