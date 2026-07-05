from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path

def generate_launch_description():
    launch_path = Path(__file__).resolve()
    project_root = launch_path.parents[3]
    if project_root.name == 'bringup' and (project_root / 'share').exists():
        project_root = project_root.parents[1]
    debug_dump_dir = str(project_root / 'sniper_debug_imgs')
    shark_ip = LaunchConfiguration('shark_ip')
    shark_port = LaunchConfiguration('shark_port')
    shark_timeout = LaunchConfiguration('shark_frame_timeout_s')
    enable_separate_decoder = LaunchConfiguration('enable_separate_decoder')

    shark_bridge_node = Node(
        package='doorlock_decoder',
        executable='shark_bridge_node',
        name='shark_bridge',
        parameters=[
            {'shark_ip': shark_ip},
            {'shark_port': shark_port},
            {'frame_timeout_s': shark_timeout},
            {'packet_size': 300}
        ],
        remappings=[
            ('/video_stream', '/video_stream/shark'),
            ('/video_format', '/video_format/shark')
        ],
        output='screen',
        emulate_tty=True
    )

    decoder_shark_node = Node(
        package='doorlock_decoder',
        executable='decoder_node',
        name='decoder_shark',
        parameters=[
            {'topic': '/video_stream/shark'},
            {'format_topic': '/video_format/shark'},
            {'use_variable_packet': True},  # 启用变长消息
            {'display': enable_separate_decoder},
            {'default_codec': 'hevc'},
            {'disable_auto_switch': True},
            {'window_title': 'Shark UDP 视频流'},
            {'width': 960},
            {'height': 540},
            {'display_scale': 1},
            {'reset_on_gap': False},
            {'gap_reset_threshold': 50},
            {'decode_error_reset_threshold': 30},
            {'crosshair_offset_x': 0},
            {'crosshair_offset_y': 0},
            {'crosshair_width': 1},
            {'debug_dump_enable': False},
            {'debug_dump_dir': debug_dump_dir}
        ],
        remappings=[
            ('/decoded_image', '/decoded_image/shark'),
        ],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        DeclareLaunchArgument('enable_separate_decoder', default_value='False'),
        shark_bridge_node,
        decoder_shark_node
    ])
