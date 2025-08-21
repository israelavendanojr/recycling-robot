from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='recycling_robot',
            executable='camera_node',
            name='camera_node',
            output='screen'
        ),
        Node(
            package='recycling_robot',
            executable='classifier_node',
            name='classifier_node',
            output='screen'
        ),
    ])