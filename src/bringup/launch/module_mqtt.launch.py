from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from pathlib import Path

def generate_launch_description():
    launch_path = Path(__file__).resolve()
    project_root = launch_path.parents[3]
    if project_root.name == 'bringup' and (project_root / 'share').exists():
        project_root = project_root.parents[1]
    debug_dump_dir = str(project_root / 'sniper_debug_imgs')

    mqtt_ip = LaunchConfiguration('mqtt_server_ip')
    mqtt_port = LaunchConfiguration('mqtt_server_port')
    robot_id = LaunchConfiguration('robot_id')
    enable_separate_decoder = LaunchConfiguration('enable_separate_decoder')
    enable_video_stream = LaunchConfiguration('enable_video_stream', default='True')

    mqtt_bridge_node = Node(
        package='doorlock_decoder',
        executable='mqtt_custom_bridge_node',
        name='mqtt_referee_bridge',
        parameters=[{
            'mqtt_server_ip': mqtt_ip,
            'mqtt_server_port': mqtt_port,
            'robot_id': robot_id,
            'enable_video_stream': enable_video_stream,
            'packet_size': 300,
            'frame_timeout_s': 1.5,
        }],
        remappings=[
            ('/video_stream', '/video_stream/mqtt'),
            ('/video_format', '/video_format/mqtt')
        ],
        output='screen',
        emulate_tty=True
    )

    decoder_mqtt_node = Node(
        package='doorlock_decoder',
        executable='decoder_node',
        name='decoder_mqtt',
        parameters=[
            {'topic': '/video_stream/mqtt'},
            {'format_topic': '/video_format/mqtt'},
            {'display': enable_separate_decoder},
            {'window_title': 'MQTT 裁判视频流'},
            {'default_codec': 'hevc'},
            {'disable_auto_switch': True},
            {'width': 300},
            {'height': 300},
            {'display_scale': 2},
            {'reset_on_gap': False},
            {'gap_reset_threshold': 30},
            {'decode_error_reset_threshold': 20},
            {'crosshair_offset_x': 0},
            {'crosshair_offset_y': 0},
            {'crosshair_width': 1},
            {'debug_dump_enable': False},
            {'debug_dump_dir': debug_dump_dir}
        ],
        remappings=[
            ('/decoded_image', '/decoded_image/local'),
        ],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(enable_video_stream)
    )

    return LaunchDescription([
        DeclareLaunchArgument('enable_separate_decoder', default_value='False'),
        DeclareLaunchArgument('enable_video_stream', default_value='True'),
        mqtt_bridge_node,
        decoder_mqtt_node
    ])
