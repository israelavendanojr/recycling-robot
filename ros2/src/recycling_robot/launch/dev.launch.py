from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('recycling_robot')
    
    # Launch arguments
    config_file = LaunchConfiguration('config_file')
    
    # Declare launch arguments
    declare_config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'camera.yaml'),
        description='Path to configuration file'
    )
    
    # Camera Node
    camera_node = Node(
        package='recycling_robot',
        executable='camera_node',
        name='camera_node',
        output='screen',
        emulate_tty=True,
        parameters=[config_file],
        remappings=[
            ('camera/image_raw', '/camera/image_raw'),
        ]
    )

    # Classifier Node
    classifier_node = Node(
        package='recycling_robot',
        executable='classifier_node',
        name='classifier_node',
        output='screen',
        emulate_tty=True,
        parameters=[config_file],
        remappings=[
            ('camera/image_raw', '/camera/image_raw'),
            ('classification_result', '/classifier/output'),
        ]
    )

    # Web Dashboard Node
    web_node = Node(
        package='recycling_robot',
        executable='web_node',
        name='web_node',
        output='screen',
        emulate_tty=True,
        parameters=[config_file],
        remappings=[
            ('camera/image_raw', '/camera/image_raw'),
            ('classification_result', '/classifier/output'),
        ]
    )

    return LaunchDescription([
        declare_config_arg,
        camera_node,
        classifier_node,
        web_node,
    ])