from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Camera Node
        Node(
            package='recycling_robot',
            executable='camera.py',
            name='camera_node',
            output='screen',
            emulate_tty=True,
            # Try device_id 0 first (typical for ArduCam on Pi)
            parameters=[{"device_id": 0, "width": 640, "height": 480, "fps": 10.0}],
        ),

        # Classifier Node
        Node(
            package='recycling_robot',
            executable='classifier.py',
            name='classifier_node',
            output='screen',
            emulate_tty=True,
        ),

        # Web Dashboard Node
        Node(
            package='recycling_robot',
            executable='web.py',
            name='web_node',
            output='screen',
            emulate_tty=True,
        ),
    ])