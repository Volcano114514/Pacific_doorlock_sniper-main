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
    enable_separate_decoder = LaunchConfiguration('enable_separate_decoder')

    decoder_uart_node = Node(
        package='doorlock_decoder',
        executable='decoder_node',
        name='decoder_uart',
        parameters=[
            {'topic': '/video_stream/uart'},
            {'format_topic': '/video_format/uart'},
            {'use_variable_packet': False},
            {'display': enable_separate_decoder},
            {'default_codec': 'h264'},
            {'disable_auto_switch': True},
            {'window_title': '串口回传调试画面'},
            {'width': 300},
            {'height': 300},
            {'display_scale': 2},
            {'reset_on_gap': False},
            # 放宽重置阈值，适配串口偶发丢包
            {'gap_reset_threshold': 50},
            {'decode_error_reset_threshold': 30},
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
        emulate_tty=True
    )

    return LaunchDescription([
        DeclareLaunchArgument('enable_separate_decoder', default_value='False'),
        decoder_uart_node
    ])
