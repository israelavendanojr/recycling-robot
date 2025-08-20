"""
Minimal launch file for TODAY's milestone.
Launches the core nodes needed for end-to-end demo.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    """Generate minimal launch description for today's demo."""
    
    # Declare launch arguments
    camera_type_arg = DeclareLaunchArgument(
        'camera_type',
        default_value='mock',
        description='Camera type: auto, pi, or mock (default: mock for development)'
    )
    
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='models/recycler.pth',
        description='Path to the trained model file (optional for mock mode)'
    )
    
    web_port_arg = DeclareLaunchArgument(
        'web_port',
        default_value='8000',
        description='Web dashboard port'
    )
    
    # Camera Node - publishes to /camera/image_raw
    camera_node = Node(
        package='recycling_robot',
        executable='camera_node',
        name='camera_node',
        parameters=[{
            'camera_type': LaunchConfiguration('camera_type'),
            'resolution_width': 640,
            'resolution_height': 480,
            'fps': 10.0,
            'frame_id': 'camera_link'
        }],
        output='screen',
        emulate_tty=True
    )
    
    # Classifier Node - provides /classify_image service
    classifier_node = Node(
        package='recycling_robot',
        executable='classifier_node',
        name='classifier_node',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'device': 'auto',
            'input_size': 224,
            'class_names': ['cardboard', 'glass', 'metal', 'plastic', 'trash']
        }],
        output='screen',
        emulate_tty=True
    )
    
    # Web Bridge Node - serves dashboard at http://localhost:8000
    web_bridge_node = Node(
        package='recycling_robot',
        executable='web_bridge_node',
        name='web_bridge_node',
        parameters=[{
            'host': '0.0.0.0',
            'port': LaunchConfiguration('web_port'),
            'enable_video_stream': True
        }],
        output='screen',
        emulate_tty=True
    )
    
    # Launch info
    launch_info = LogInfo(
        msg=[
            '\n🚀 STARTING ROS 2 RECYCLING ROBOT - MINIMAL DEMO\n',
            '='*60 + '\n',
            'Camera Type: ', LaunchConfiguration('camera_type'), '\n',
            'Model Path:  ', LaunchConfiguration('model_path'), '\n', 
            'Dashboard:   http://localhost:', LaunchConfiguration('web_port'), '/\n',
            '='*60 + '\n',
            'Expected Flow:\n',
            '  1. Camera publishes frames to /camera/image_raw\n',
            '  2. Classifier provides service at /classify_image\n',
            '  3. Web dashboard shows live feed + stats\n',
            '  4. Open http://localhost:8000 to see dashboard\n',
            '\nPress Ctrl+C to stop all nodes.\n'
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        camera_type_arg,
        model_path_arg,
        web_port_arg,
        
        # Info message
        launch_info,
        
        # Core nodes
        camera_node,
        classifier_node,
        web_bridge_node,
    ])