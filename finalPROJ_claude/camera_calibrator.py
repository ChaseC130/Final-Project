#!/usr/bin/env python3
"""
Camera Calibration Tool for Lane Detection
Use this to verify your camera setup and tune color detection parameters
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraCalibrator(Node):
    def __init__(self):
        super().__init__('camera_calibrator')
        
        # Parameters
        self.declare_parameter('camera_topic', '/camera/image_raw')
        camera_topic = self.get_parameter('camera_topic').value
        
        self.bridge = CvBridge()
        
        # HSV ranges (adjustable with trackbars)
        self.white_lower = np.array([0, 0, 200])
        self.white_upper = np.array([180, 30, 255])
        self.yellow_lower = np.array([20, 100, 100])
        self.yellow_upper = np.array([30, 255, 255])
        
        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )
        
        # Create window and trackbars
        cv2.namedWindow('Camera Calibration')
        cv2.namedWindow('White Detection')
        cv2.namedWindow('Yellow Detection')
        
        # White detection trackbars
        cv2.createTrackbar('White H Min', 'White Detection', 0, 180, self.on_trackbar)
        cv2.createTrackbar('White H Max', 'White Detection', 180, 180, self.on_trackbar)
        cv2.createTrackbar('White S Min', 'White Detection', 0, 255, self.on_trackbar)
        cv2.createTrackbar('White S Max', 'White Detection', 30, 255, self.on_trackbar)
        cv2.createTrackbar('White V Min', 'White Detection', 200, 255, self.on_trackbar)
        cv2.createTrackbar('White V Max', 'White Detection', 255, 255, self.on_trackbar)
        
        # Yellow detection trackbars
        cv2.createTrackbar('Yellow H Min', 'Yellow Detection', 20, 180, self.on_trackbar)
        cv2.createTrackbar('Yellow H Max', 'Yellow Detection', 30, 180, self.on_trackbar)
        cv2.createTrackbar('Yellow S Min', 'Yellow Detection', 100, 255, self.on_trackbar)
        cv2.createTrackbar('Yellow S Max', 'Yellow Detection', 255, 255, self.on_trackbar)
        cv2.createTrackbar('Yellow V Min', 'Yellow Detection', 100, 255, self.on_trackbar)
        cv2.createTrackbar('Yellow V Max', 'Yellow Detection', 255, 255, self.on_trackbar)
        
        # ROI height
        cv2.createTrackbar('ROI Height %', 'Camera Calibration', 30, 100, self.on_trackbar)
        
        self.get_logger().info('Camera Calibrator Started')
        self.get_logger().info('Adjust trackbars to tune color detection')
        self.get_logger().info('Press Q to quit and print optimal values')
    
    def on_trackbar(self, val):
        """Trackbar callback (required but not used)"""
        pass
    
    def image_callback(self, msg):
        """Process and display camera image with detection results"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width = cv_image.shape[:2]
            
            # Get ROI height from trackbar
            roi_percent = cv2.getTrackbarPos('ROI Height %', 'Camera Calibration')
            roi_start = int(height * (1 - roi_percent / 100.0))
            roi = cv_image[roi_start:, :]
            
            # Get trackbar values for white detection
            self.white_lower[0] = cv2.getTrackbarPos('White H Min', 'White Detection')
            self.white_upper[0] = cv2.getTrackbarPos('White H Max', 'White Detection')
            self.white_lower[1] = cv2.getTrackbarPos('White S Min', 'White Detection')
            self.white_upper[1] = cv2.getTrackbarPos('White S Max', 'White Detection')
            self.white_lower[2] = cv2.getTrackbarPos('White V Min', 'White Detection')
            self.white_upper[2] = cv2.getTrackbarPos('White V Max', 'White Detection')
            
            # Get trackbar values for yellow detection
            self.yellow_lower[0] = cv2.getTrackbarPos('Yellow H Min', 'Yellow Detection')
            self.yellow_upper[0] = cv2.getTrackbarPos('Yellow H Max', 'Yellow Detection')
            self.yellow_lower[1] = cv2.getTrackbarPos('Yellow S Min', 'Yellow Detection')
            self.yellow_upper[1] = cv2.getTrackbarPos('Yellow S Max', 'Yellow Detection')
            self.yellow_lower[2] = cv2.getTrackbarPos('Yellow V Min', 'Yellow Detection')
            self.yellow_upper[2] = cv2.getTrackbarPos('Yellow V Max', 'Yellow Detection')
            
            # Convert ROI to HSV
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            
            # Create masks
            white_mask = cv2.inRange(hsv, self.white_lower, self.white_upper)
            yellow_mask = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
            
            # Apply morphological operations
            kernel = np.ones((3, 3), np.uint8)
            white_mask_clean = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)
            white_mask_clean = cv2.morphologyEx(white_mask_clean, cv2.MORPH_OPEN, kernel)
            
            yellow_mask_clean = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)
            yellow_mask_clean = cv2.morphologyEx(yellow_mask_clean, cv2.MORPH_OPEN, kernel)
            
            # Create colored overlays
            white_overlay = roi.copy()
            white_overlay[white_mask_clean > 0] = [255, 0, 0]  # Blue for white lines
            
            yellow_overlay = roi.copy()
            yellow_overlay[yellow_mask_clean > 0] = [0, 255, 0]  # Green for yellow lines
            
            # Combined result
            result = roi.copy()
            result[white_mask_clean > 0] = [255, 0, 0]
            result[yellow_mask_clean > 0] = [0, 255, 0]
            
            # Add ROI indicator to full image
            display_image = cv_image.copy()
            cv2.rectangle(display_image, (0, roi_start), (width, height), (0, 255, 0), 2)
            cv2.putText(display_image, f'ROI: {roi_percent}%', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Add detection info to result
            white_pixels = np.sum(white_mask_clean > 0)
            yellow_pixels = np.sum(yellow_mask_clean > 0)
            cv2.putText(result, f'White pixels: {white_pixels}', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(result, f'Yellow pixels: {yellow_pixels}', (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Display images
            cv2.imshow('Camera Calibration', display_image)
            cv2.imshow('White Detection', white_overlay)
            cv2.imshow('Yellow Detection', yellow_overlay)
            cv2.imshow('Combined Result', result)
            
            # Check for quit key
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == ord('Q'):
                self.print_values()
                rclpy.shutdown()
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def print_values(self):
        """Print current HSV values for use in main code"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('OPTIMAL VALUES - Copy these to your lane_follower.py:')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'White line detection:')
        self.get_logger().info(f'  lower_white = np.array([{self.white_lower[0]}, {self.white_lower[1]}, {self.white_lower[2]}])')
        self.get_logger().info(f'  upper_white = np.array([{self.white_upper[0]}, {self.white_upper[1]}, {self.white_upper[2]}])')
        self.get_logger().info('')
        self.get_logger().info(f'Yellow line detection:')
        self.get_logger().info(f'  lower_yellow = np.array([{self.yellow_lower[0]}, {self.yellow_lower[1]}, {self.yellow_lower[2]}])')
        self.get_logger().info(f'  upper_yellow = np.array([{self.yellow_upper[0]}, {self.yellow_upper[1]}, {self.yellow_upper[2]}])')
        self.get_logger().info('')
        roi_percent = cv2.getTrackbarPos('ROI Height %', 'Camera Calibration')
        self.get_logger().info(f'ROI height: {roi_percent / 100.0:.2f}')
        self.get_logger().info('=' * 60)


def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibrator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
