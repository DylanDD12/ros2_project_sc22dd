#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PoseStamped
from cv_bridge import CvBridge
from std_msgs.msg import String
import random
import math

class RobotNode(Node):
    def __init__(self):
        super().__init__('robotnode')
        self.bridge = CvBridge()
        
        # Subscriptions and Publishers
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'goals', 10)
        self.cancel_pub = self.create_publisher(String, 'cancelblue', 10)
        
        # State Vars
        self.sensitivity = 10
        self.detected = {"green": False, "blue": False, "red": False}
        self.blue_area = 0.0
        self.blue_center = None
        self.image_width = None
        
        # Corner bounds
        self.x_range = (-9.5, 7)
        self.y_range = (-13.5, 4.5)
        
        # Timer
        self.create_timer(0.1, self.control_callback)

    def image_callback(self, img_msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return

        self.image_width = frame.shape[1]
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Green
        lower_green = np.array([60 - self.sensitivity, 100, 100])
        upper_green = np.array([60 + self.sensitivity, 255, 255])
        green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.detected["green"] = False
        if green_contours:
            largest_green = max(green_contours, key=cv2.contourArea)
            if cv2.contourArea(largest_green) > 100:
                # Get center
                (x_green, y_green), radius_green = cv2.minEnclosingCircle(largest_green)
                # Draw box
                gx, gy, gw, gh = cv2.boundingRect(largest_green)
                cv2.rectangle(frame, (gx, gy), (gx + gw, gy + gh), (0, 255, 0), 2)
                cv2.putText(frame, "Green", (gx + gw, gy + gh), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                self.detected["green"] = True

        # Blue
        lower_blue = np.array([110 - self.sensitivity, 100, 100])
        upper_blue = np.array([110 + self.sensitivity, 255, 255])
        blue_mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)
        blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.detected["blue"] = False
        if blue_contours:
            largest_blue = max(blue_contours, key=cv2.contourArea)
            area_blue = cv2.contourArea(largest_blue)
            if area_blue > 100:
                self.blue_area = area_blue
                # Get center
                (x_blue, y_blue), radius_blue = cv2.minEnclosingCircle(largest_blue)
                self.blue_center = (int(x_blue), int(y_blue))
                # Draw box
                bx, by, bw, bh = cv2.boundingRect(largest_blue)
                cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), (255, 0, 0), 2)
                cv2.putText(frame, "Blue", (bx + bw, by + bh), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                self.detected["blue"] = True

                # Publish cancel message
                cancel_msg = String()
                cancel_msg.data = "cancel"
                self.cancel_pub.publish(cancel_msg)

        # Red
        lower_red = np.array([0 - self.sensitivity, 100, 100])
        upper_red = np.array([0 + self.sensitivity, 255, 255])
        red_mask = cv2.inRange(hsv_frame, lower_red, upper_red)
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.detected["red"] = False
        if red_contours:
            largest_red = max(red_contours, key=cv2.contourArea)
            if cv2.contourArea(largest_red) > 100:
                (x_red, y_red), radius_red = cv2.minEnclosingCircle(largest_red)
                # Draw box
                rx, ry, rw, rh = cv2.boundingRect(largest_red)
                cv2.rectangle(frame, (rx, ry), (rx + rw, ry + rh), (0, 0, 255), 2)
                cv2.putText(frame, "Red", (rx + rw, ry + rh), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                self.detected["red"] = True

        # Display in a window
        cv2.namedWindow("Vision", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Vision", 320, 240)
        cv2.imshow("Vision", frame)
        cv2.waitKey(1)

    def blue_approach(self):
        desired_area = 570000
        if self.image_width is None or self.blue_center is None:
            self.stop_movement()
            return

        img_center = self.image_width // 2  # Center
        blue_x, _ = self.blue_center
        offset = blue_x - img_center
        angular_z = -0.002 * offset  # Angular velocity

        tolerance = 18000  # Area tolerance
        if self.blue_area < desired_area - tolerance:
            linear_x = 0.2
        elif self.blue_area > desired_area + tolerance:
            linear_x = -0.1
        else:
            linear_x = 0.0

        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.vel_pub.publish(cmd)

    def stop_movement(self):
        # Stop robot
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.vel_pub.publish(stop_cmd)

    def pick_random_goal(self):
        # Give random goals
        x_goal = random.uniform(self.x_range[0], self.x_range[1])
        y_goal = random.uniform(self.y_range[0], self.y_range[1])
        yaw = random.uniform(-math.pi, math.pi)

        goal = PoseStamped()
        goal.pose.position.x = x_goal
        goal.pose.position.y = y_goal
        goal.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.orientation.w = math.cos(yaw / 2)
        self.goal_pub.publish(goal)

    def control_callback(self):
        if self.detected["blue"]:
            self.blue_approach()
        else:
            self.pick_random_goal()

def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down RobotNode.")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
