from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_project_sc22dd',
            executable='blue_detector',
            name='blue_detector'
        ),
        Node(
            package='ros2_project_sc22dd',
            executable='motion_planner',
            name='motion_planner'
        ),
    ])