from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    backend_url = LaunchConfiguration('backend_url')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'backend_url',
            default_value='http://backend:8000',
            description='Backend API base URL'
        ),
        
        # Camera node
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
        
        # Classifier node
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
    ])