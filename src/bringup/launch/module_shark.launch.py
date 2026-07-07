from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    # 参数
    declare_sh_ip = DeclareLaunchArgument('sh_ip', default_value='192.168.12.1')
    declare_sh_port = DeclareLaunchArgument('sh_port', default_value='3334')
    declare_timeout = DeclareLaunchArgument('shark_frame_timeout_s', default_value='1.0')
    declare_sep_decoder = DeclareLaunchArgument('enable_separate_decoder', default_value='False')

    sh_ip = LaunchConfiguration('sh_ip')
    sh_port = LaunchConfiguration('sh_port')
    timeout = LaunchConfiguration('shark_frame_timeout_s')
    sep_decoder = LaunchConfiguration('enable_separate_decoder')

    # UDP收发桥接节点 shark_bridge_node.py
    shark_bridge_node = Node(
        package='doorlock_sniper',
        executable='shark_bridge_node',
        name='shark_bridge',
        parameters=[
            {'target_ip': sh_ip},
            {'udp_port': sh_port},
            {'frame_timeout_s': timeout}
        ],
        remappings=[
            ('/shark_out', '/video_stream/shark'),
            ('/shark_in', '/video_stream')
        ],
        output='screen',
        emulate_tty=True
    )

    # Shark UDP解码器
    shark_decoder_node = Node(
        package='doorlock_decoder',
        executable='decoder_node',
        name='decoder_shark',
        parameters=[
            {'topic': '/video_stream/shark'},
            {'format_topic': '/video_format/shark'},
            {'use_variable_packet': True},
            {'display': sep_decoder},
            {'default_codec': 'h264'},
            {'gap_reset_threshold': 50},
            {'decode_error_reset_threshold': 30},
            {'window_title': 'Shark UDP画面'}
        ],
        remappings=[
            ('/decoded_image', '/decoded_image/shark')
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_sh_ip,
        declare_sh_port,
        declare_timeout,
        declare_sep_decoder,
        shark_bridge_node,
        shark_decoder_node
    ])
