#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError

class BlueDetector(Node):
    def __init__(self):
        super().__init__('blue_detector')
        # Initialize CvBridge for converting images.
        self.bridge = CvBridge()
        # Subscribe to the camera feed topic.
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        # Publisher for blue detection flag.
        self.blue_detected_pub = self.create_publisher(Bool, '/blue_box_detected', 10)
        # Publisher for the blue box center.
        self.blue_center_pub = self.create_publisher(Point, '/blue_box_center', 10)
        
        # Sensitivity value for HSV thresholding.
        self.sensitivity = 10  # adjust as needed
        # Threshold for minimum contour area to consider a detection valid.
        self.min_area = 500
        
    def image_callback(self, msg):
        try:
            # Convert the ROS Image message to an OpenCV image.
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        # Convert the image from BGR to HSV.
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define HSV range for blue. (Blue is typically around 120 in OpenCV)
        lower_blue = np.array([120 - self.sensitivity, 100, 100])
        upper_blue = np.array([120 + self.sensitivity, 255, 255])
        
        # Create a mask for blue objects.
        blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)
        
        # Optionally, you can smooth the mask to reduce noise:
        blue_mask = cv2.GaussianBlur(blue_mask, (5, 5), 0)
        
        # Find contours in the mask.
        contours, _ = cv2.findContours(blue_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        
        blue_detected = False
        detected_center = None
        
        if contours:
            # Find the largest contour.
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            if area > self.min_area:
                blue_detected = True
                # Calculate the center of the contour using moments.
                M = cv2.moments(largest_contour)
                if M['m00'] != 0:
                    cX = int(M['m10'] / M['m00'])
                    cY = int(M['m01'] / M['m00'])
                    detected_center = (cX, cY)
                    # Optionally, draw a circle and bounding box on the image.
                    cv2.circle(image, (cX, cY), 5, (255, 0, 0), -1)
                    x, y, w, h = cv2.boundingRect(largest_contour)
                    cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)
                else:
                    self.get_logger().warn("Zero division error in moment calculation")
        
        # Publish detection flag.
        flag_msg = Bool()
        flag_msg.data = blue_detected
        self.blue_detected_pub.publish(flag_msg)
        
        # Publish blue box center if detected.
        if blue_detected and detected_center is not None:
            point_msg = Point()
            point_msg.x = detected_center[0]
            point_msg.y = detected_center[1]
            point_msg.z = 0.0  # For 2D, z remains 0.
            self.blue_center_pub.publish(point_msg)
        
        # Optionally display the original image with overlay.
        cv2.namedWindow('Camera Feed', cv2.WINDOW_NORMAL)
        cv2.imshow('Camera Feed', image)
        cv2.resizeWindow('Camera Feed', 320, 240)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = BlueDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down blue detector node.")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

