from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    backend_url = LaunchConfiguration('backend_url')
    web_host = LaunchConfiguration('web_host')
    web_port = LaunchConfiguration('web_port')

    return LaunchDescription([
        # --- Args ---
        DeclareLaunchArgument(
            'backend_url',
            default_value='http://backend:8000',
            description='Backend API base URL'
        ),
        DeclareLaunchArgument(
            'web_host',
            default_value='0.0.0.0',
            description='Host/interface for the Web Dashboard node'
        ),
        DeclareLaunchArgument(
            'web_port',
            default_value='8080',   # avoid clash with backend:8000
            description='Port for the Web Dashboard node'
        ),

        # --- Camera node ---
        Node(
            package='recycling_robot',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'device_id': 0,
                'fps': 10.0
            }]
        ),

        # --- Classifier node ---
        Node(
            package='recycling_robot',
            executable='classifier_node',
            name='classifier_node',
            output='screen',
            parameters=[{
                'api_base_url': backend_url,
                'inference_interval': 3.0,
                'confidence_threshold': 0.7
            }]
        ),

        # --- Web Dashboard node ---
        Node(
            package='recycling_robot',
            executable='web_node',   
            name='simple_web',
            output='screen',
            parameters=[{
                'host': web_host,
                'port': web_port
            }]
        ),
    ])
