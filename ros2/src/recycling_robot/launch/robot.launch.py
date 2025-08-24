from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
import os

def generate_launch_description():
    # Launch arguments
    use_mock_camera_arg = DeclareLaunchArgument(
        'use_mock_camera',
        default_value='true',
        description='Use mock camera instead of real camera'
    )
    
    backend_url_arg = DeclareLaunchArgument(
        'backend_url',
        default_value='http://backend:8000',
        description='Backend API base URL'
    )
    
    web_host_arg = DeclareLaunchArgument(
        'web_host',
        default_value='0.0.0.0',
        description='Host/interface for the Web Dashboard node'
    )
    
    web_port_arg = DeclareLaunchArgument(
        'web_port',
        default_value='8080',
        description='Port for the Web Dashboard node'
    )
    
    # Get the launch directory
    launch_dir = os.path.join(os.path.dirname(__file__))
    
    return LaunchDescription([
        # Launch arguments
        use_mock_camera_arg,
        backend_url_arg,
        web_host_arg,
        web_port_arg,
        
        # --- Mock Camera node (for testing) ---
        Node(
            package='recycling_robot',
            executable='mock_camera_node',
            name='mock_camera_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_mock_camera')),
            parameters=[{
                'image_folder': 'test_images',
                'publish_rate': 3.0,
                'image_quality': 85,
                'auto_fallback': True
            }]
        ),
        
        # --- Real Camera node (alternative) ---
        Node(
            package='recycling_robot',
            executable='camera_node',
            name='camera_node',
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('use_mock_camera')),
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
                'api_base_url': LaunchConfiguration('backend_url'),
                'inference_interval': 3.0,
                'confidence_threshold': 0.7
            }]
        ),

        # --- Sorting node ---
        Node(
            package='recycling_robot',
            executable='sorting_node',
            name='sorting_node',
            output='screen',
            parameters=[{
                'sorting_delay': 1.0
            }]
        ),

        # --- Web Dashboard node ---
        Node(
            package='recycling_robot',
            executable='web_node',   
            name='simple_web',
            output='screen',
            parameters=[{
                'host': LaunchConfiguration('web_host'),
                'port': LaunchConfiguration('web_port')
            }]
        ),
    ])
