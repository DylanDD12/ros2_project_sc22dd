import math
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool

class MotionPlanner(Node):
    def __init__(self):
        super().__init__('motion_planner')

        # Action client for navigation goals.
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscribers for blue box detection.
        self.create_subscription(Bool, 'blue_box_detected', self.blue_detected_callback, 10)
        self.create_subscription(Point, 'blue_box_center', self.blue_box_center_callback, 10)
        self.blue_detected = False
        self.blue_box_center = None  # type: Point

        # Candidate poses for exploration (example values adjust as needed)
        self.candidate_goals = [
            {'x': 1.0, 'y': 1.0, 'yaw': 0.0},
            {'x': 1.0, 'y': 4.0, 'yaw': math.pi/2},
            {'x': 4.0, 'y': 4.0, 'yaw': math.pi},
            {'x': 4.0, 'y': 1.0, 'yaw': -math.pi/2},
        ]
        self.current_goal_index = 0

        # Flag to indicate if we have an active navigation goal
        self.navigation_active = False

        # Timer to check and send next goal
        timer_period = 5.0  # seconds between candidate goal submissions
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def blue_detected_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Blue box detected!")
            self.blue_detected = True
        else:
            self.blue_detected = False

    def blue_box_center_callback(self, msg: Point):
        # Update the last known center of the blue box.
        self.blue_box_center = msg
        self.get_logger().debug(f"Blue box center updated: x={msg.x}, y={msg.y}")

    def timer_callback(self):
        # If blue box detected and we have a center coordinate, override exploration.
        if self.blue_detected and self.blue_box_center is not None:
            # Stop exploration and send final approach goal.
            if not self.navigation_active:
                self.get_logger().info("Overriding exploration with final approach to blue box.")
                # Compute final goal: approach the blue box so that robot stops 1 meter away.
                # Here we assume an approach yaw of 0.0 (i.e. coming from the front)
                final_goal = self.compute_final_goal(
                    self.blue_box_center.x, self.blue_box_center.y,
                    approach_yaw=0.0, stop_distance=1.0)
                self.send_goal(final_goal['x'], final_goal['y'], final_goal['yaw'])
            return

        # Otherwise, if no blue detection and no active goal, send the next candidate goal.
        if not self.navigation_active:
            candidate = self.candidate_goals[self.current_goal_index]
            self.get_logger().info(
                f"Exploring candidate pose {self.current_goal_index}: "
                f"(x={candidate['x']}, y={candidate['y']}, yaw={candidate['yaw']})"
            )
            self.send_goal(candidate['x'], candidate['y'], candidate['yaw'])
            # Cycle through candidate list.
            self.current_goal_index = (self.current_goal_index + 1) % len(self.candidate_goals)

    def compute_final_goal(self, box_center_x, box_center_y, approach_yaw, stop_distance=1.0):
        # Compute target coordinates by offsetting the box center by the stop distance along the approach direction.
        goal_x = box_center_x - stop_distance * math.cos(approach_yaw)
        goal_y = box_center_y - stop_distance * math.sin(approach_yaw)
        self.get_logger().info(
            f"Final goal computed: x={goal_x:.2f}, y={goal_y:.2f} (approach_yaw={approach_yaw})"
        )
        return {'x': goal_x, 'y': goal_y, 'yaw': approach_yaw}

    def send_goal(self, x, y, yaw):
        self.navigation_active = True

        goal_msg = NavigateToPose.Goal()
        # Set position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        # Convert yaw to quaternion (2D: only z and w)
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(f"Sending navigation goal: x={x}, y={y}, yaw={yaw}")
        self._action_client.wait_for_server()
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            self.navigation_active = False
            return

        self.get_logger().info("Goal accepted")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Navigation result: {result}")
        # Mark navigation as inactive to allow sending a new goal.
        self.navigation_active = False

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback: {feedback}")

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()