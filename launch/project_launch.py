from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nav_node = Node(
        package='ros2_project_sc22dd',       
        executable='nav_node', 
        name='nav_node',
        output='screen'
    )
    vision_node = Node(
        package='ros2_project_sc22dd',      
        executable='robot_node', 
        name='robot_node',
        output='screen'
    )

    return LaunchDescription([
        nav_node,
        vision_node
    ])