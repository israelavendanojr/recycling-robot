from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    camera_type_arg = DeclareLaunchArgument('camera_type', default_value='auto')
    model_path_arg = DeclareLaunchArgument('model_path', default_value='models/recycler.pth')
    web_port_arg   = DeclareLaunchArgument('web_port',   default_value='8000')
    debug_mode_arg = DeclareLaunchArgument('debug_mode', default_value='true')

    set_opencv_debug = SetEnvironmentVariable(name='OPENCV_VIDEOIO_DEBUG', value='1')
    set_opencv_log   = SetEnvironmentVariable(name='OPENCV_LOG_LEVEL',     value='INFO')

    camera_node = Node(
        package='recycling_robot',
        executable='camera_node',
        name='camera_node',
        parameters=[{
            'camera_type': LaunchConfiguration('camera_type'),
            'resolution_width': 640,
            'resolution_height': 480,
            'fps': 10.0,
            'frame_id': 'camera_link',
            'debug_mode': LaunchConfiguration('debug_mode'),
        }],
        output='screen',
        emulate_tty=True,
    )

    classifier_node = Node(
        package='recycling_robot',
        executable='classifier_node',
        name='classifier_node',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'device': 'auto',
            'input_size': 224,
            'class_names': ['cardboard', 'glass', 'metal', 'plastic', 'trash'],
        }],
        output='screen',
        emulate_tty=True,
    )

    web_bridge_node = Node(
        package='recycling_robot',
        executable='web_bridge_node',
        name='web_bridge_node',
        parameters=[{
            'host': '0.0.0.0',
            'port': LaunchConfiguration('web_port'),
            'enable_video_stream': True,
            'auto_classify_interval': 3.0,
        }],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        # args
        camera_type_arg, model_path_arg, web_port_arg, debug_mode_arg,
        # ✅ env
        set_opencv_debug, set_opencv_log,
        # nodes
        camera_node, classifier_node, web_bridge_node,
    ])
