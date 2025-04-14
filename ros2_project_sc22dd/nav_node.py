import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class NavNode(Node):
    def __init__(self):
        super().__init__('navnode')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscriptions
        self.create_subscription(PoseStamped, 'goals', self.goal_callback, 10)
        self.create_subscription(String, 'cancelblue', self.cancel_callback, 10)

        self.active_goal_handle = None
        self.goal_active = False

    def goal_callback(self, goal_msg: PoseStamped):
        if self.goal_active:
            self.get_logger().info("A navigation goal is already active. New goal ignored.")
            return

        # Create new goal message
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose.header.frame_id = "map"
        nav_goal.pose.header.stamp = self.get_clock().now().to_msg()
        nav_goal.pose.pose.position.x = goal_msg.pose.position.x
        nav_goal.pose.pose.position.y = goal_msg.pose.position.y
        nav_goal.pose.pose.orientation.z = goal_msg.pose.orientation.z
        nav_goal.pose.pose.orientation.w = goal_msg.pose.orientation.w

        # Send to action server
        self.get_logger().info(f"Sending goal: ({nav_goal.pose.pose.position.x}, {nav_goal.pose.pose.position.y})")
        self._action_client.wait_for_server()
        self.goal_active = True
        send_future = self._action_client.send_goal_async(nav_goal, feedback_callback=self.feedback_callback)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        # Handle server response
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Navigation goal rejected.")
            self.goal_active = False
            return

        self.get_logger().info("Navigation goal accepted.")
        self.active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        # Log result
        result = future.result().result
        self.get_logger().info(f"Navigation result: {result}")
        self.goal_active = False
        self.active_goal_handle = None

    def feedback_callback(self, feedback_msg):
        # Log feedback
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback: {feedback}")

    def cancel_callback(self, cancel_msg: String):
        # Cancel curent goal
        if self.active_goal_handle:
            self.get_logger().info("Received cancellation request. Cancelling goal...")
            cancel_future = self.active_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_response_callback)
        else:
            self.get_logger().info("Cancellation requested but no active goal is present.")

    def cancel_response_callback(self, future):
        # Log cancel result
        cancel_result = future.result()
        self.get_logger().info("Goal cancellation complete.")
        self.goal_active = False
        self.active_goal_handle = None

def main(args=None):
    rclpy.init(args=args)
    node = NavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down NavNode.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
