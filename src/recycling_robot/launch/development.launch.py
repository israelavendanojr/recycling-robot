#!/usr/bin/env python3
"""
Development launch file for recycling robot.
Starts core nodes with development-friendly settings.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    """Generate launch description for development setup."""
    
    # Declare launch arguments
    camera_type_arg = DeclareLaunchArgument(
        'camera_type',
        default_value='mock',
        description='Camera type: auto, pi, arducam, or mock'
    )
    
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='models/recycler.pth',
        description='Path to the trained model file'
    )
    
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='640x480',
        description='Camera resolution in WIDTHxHEIGHT format'
    )
    
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='auto',
        description='Computing device: cpu, cuda, or auto'
    )
    
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='10.0',
        description='Camera frame rate'
    )
    
    start_web_bridge_arg = DeclareLaunchArgument(
        'start_web_bridge',
        default_value='true',
        description='Whether to start web bridge node'
    )
    
    # Parse resolution
    resolution = LaunchConfiguration('resolution')
    
    # Camera Node
    camera_node = Node(
        package='recycling_robot',
        executable='camera_node',
        name='camera_node',
        parameters=[{
            'camera_type': LaunchConfiguration('camera_type'),
            'resolution_width': 640,  # TODO: Parse from resolution arg
            'resolution_height': 480,
            'fps': LaunchConfiguration('fps'),
            'device_id': 0,
            'frame_id': 'camera_link'
        }],
        output='screen',
        emulate_tty=True
    )
    
    # Classifier Node
    classifier_node = Node(
        package='recycling_robot',
        executable='classifier_node',
        name='classifier_node',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'device': LaunchConfiguration('device'),
            'input_size': 224,
            'class_names': ['cardboard', 'glass', 'metal', 'plastic', 'trash']
        }],
        output='screen',
        emulate_tty=True
    )
    
    # Stats Monitor Node
    stats_monitor_node = Node(
        package='recycling_robot',
        executable='stats_monitor_node',
        name='stats_monitor_node',
        output='screen',
        emulate_tty=True
    )
    
    # Web Bridge Node (conditional)
    web_bridge_node = Node(
        package='recycling_robot',
        executable='web_bridge_node',
        name='web_bridge_node',
        parameters=[{
            'host': '0.0.0.0',
            'port': 8000,
            'enable_video_stream': True
        }],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('start_web_bridge'))
    )
    
    # Safety Node
    safety_node = Node(
        package='recycling_robot',
        executable='safety_node',
        name='safety_node',
        parameters=[{
            'monitor_interval': 5.0,
            'node_timeout': 10.0
        }],
        output='screen',
        emulate_tty=True
    )
    
    # Launch info
    launch_info = LogInfo(
        msg=[
            'Starting Recycling Robot in DEVELOPMENT mode',
            TextSubstitution(text='\n  Camera: '),
            LaunchConfiguration('camera_type'),
            TextSubstitution(text='\n  Model: '),
            LaunchConfiguration('model_path'),
            TextSubstitution(text='\n  Resolution: '),
            LaunchConfiguration('resolution'),
            TextSubstitution(text='\n  Device: '),
            LaunchConfiguration('device'),
            TextSubstitution(text='\n  Web Dashboard: http://localhost:8000\n')
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        camera_type_arg,
        model_path_arg,
        resolution_arg,
        device_arg,
        fps_arg,
        start_web_bridge_arg,
        
        # Info message
        launch_info,
        
        # Nodes
        camera_node,
        classifier_node,
        stats_monitor_node,
        web_bridge_node,
        safety_node,
    ])