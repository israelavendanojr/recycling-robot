from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory for default paths
    package_share_dir = get_package_share_directory('recycling_robot')
    default_images = os.path.join(package_share_dir, 'test_images')
    
    # Launch arguments
    use_real_camera_arg = DeclareLaunchArgument(
        'use_real_camera',
        default_value='false',
        description='Use real camera instead of mock camera'
    )
    
    image_folder_arg = DeclareLaunchArgument(
        'image_folder',
        default_value=default_images,
        description='Folder of test images for MockCameraNode'
    )
    
    backend_url_arg = DeclareLaunchArgument(
        'backend_url',
        default_value='http://backend:8000',
        description='Backend API base URL'
    )
    
    # Get the launch directory
    launch_dir = os.path.join(os.path.dirname(__file__))
    
    return LaunchDescription([
        # Launch arguments
        use_real_camera_arg,
        image_folder_arg,
        backend_url_arg,
        
        # --- Mock Camera node (default) ---
        Node(
            package='recycling_robot',
            executable='mock_camera_node',
            name='mock_camera_node',
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('use_real_camera')),
            parameters=[{
                'image_folder': LaunchConfiguration('image_folder'),
                'publish_rate': 3.0,
                'image_quality': 85,
                'auto_fallback': True
            }]
        ),
        
        # --- Real Camera node (when use_real_camera=true) ---
        Node(
            package='recycling_robot',
            executable='camera_node',
            name='camera_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_real_camera')),
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
                'confidence_threshold': 0.7,
                'model_path': 'src/recycling_robot/recycling_robot/models/recycler.pt'
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
    ])
