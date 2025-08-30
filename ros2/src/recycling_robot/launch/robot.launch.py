from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory for default paths
    package_share_dir = get_package_share_directory('recycling_robot')
    
    # Launch arguments
    backend_url_arg = DeclareLaunchArgument(
        'backend_url',
        default_value='http://backend:8000',
        description='Backend API base URL'
    )
    
    # Get the launch directory
    launch_dir = os.path.join(os.path.dirname(__file__))
    
    return LaunchDescription([
        # Launch arguments
        backend_url_arg,
        
        # Startup logging
        ExecuteProcess(
            cmd=['echo', '[LAUNCH] 🚀 Starting Recycling Robot with Manual Capture Pipeline...'],
            name='startup_log_1',
            output='screen'
        ),
        
        # --- Pipeline Coordinator node (start first) ---
        ExecuteProcess(
            cmd=['echo', '[LAUNCH] 🔄 Starting Pipeline Coordinator...'],
            name='startup_log_2',
            output='screen'
        ),
        
        Node(
            package='recycling_robot',
            executable='pipeline_coordinator_node',
            name='pipeline_coordinator_node',
            output='screen',
            parameters=[{
                'timeout_seconds': 10.0,
                'state_file_path': '/tmp/pipeline_state.json'
            }]
        ),
        
        # Wait for coordinator to start
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=['echo', '[LAUNCH] 📷 Starting Camera and Processing Nodes...'],
                    name='startup_log_3',
                    output='screen'
                )
            ]
        ),
        
        # --- Real Camera node ---
        Node(
            package='recycling_robot',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[os.path.join(package_share_dir, 'config', 'camera.yaml')]
        ),

        # --- Classifier node ---
        Node(
            package='recycling_robot',
            executable='classifier_node',
            name='classifier_node',
            output='screen',
            parameters=[{
                'api_base_url': LaunchConfiguration('backend_url'),
                'inference_interval': 0.0,  # Disable auto-classification timer
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

        # --- Capture Server (HTTP trigger for dashboard) ---
        Node(
            package='recycling_robot',
            executable='capture_server',
            name='capture_server',
            output='screen'
        ),
        
        # Key input is handled by host-side key_listener.py
        # No key_input_node needed in ROS2 launch
        
        # Simple pipeline status check
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=['echo', '[LAUNCH] ✅ Pipeline coordinator ready - check status with: ros2 topic echo /pipeline/state'],
                    name='pipeline_ready',
                    output='screen'
                )
            ]
        ),
    ])
