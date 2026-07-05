from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from pathlib import Path

def generate_launch_description():
    launch_path = Path(__file__).resolve()
    project_root = launch_path.parents[3]
    if project_root.name == 'bringup' and (project_root / 'share').exists():
        project_root = project_root.parents[1]

    enable_separate_decoder = LaunchConfiguration('enable_separate_decoder')

    # 固定参数
    debug_dump_dir = str(project_root / 'sniper_debug_imgs')
    debug_dump_enable = False
    debug_dump_every_n_frames = 1
    dump_save_raw = False
    dump_save_roi = True
    dump_save_static = False
    dump_save_final = True
    dump_save_decoder = True

    target_bitrate_kbytes = 10.0
    hard_max_bitrate_kbytes = 14.0
    target_bitrate_kbps = int(target_bitrate_kbytes * 8.0)
    x264_preset = 'veryslow'
    encode_size = 300

    wb_auto = 'Off'
    wb_red = 1.7383
    wb_blue = 2.0

    # 组件容器：大恒相机 + 编码器
    encoder_container = ComposableNodeContainer(
        name='sniper_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='daheng_camera',
                plugin='daheng_camera::DahengCameraNode',
                name='daheng_camera',
                parameters=[
                    {'exposure_time': 15000.0},
                    {'gain': 10.0},
                    {'whitebalance_auto': wb_auto},
                    {'wb_red_ratio': wb_red},
                    {'wb_blue_ratio': wb_blue},
                    {'wb_green_ratio': 1.0},
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='doorlock_sniper',
                plugin='doorlock_sniper::VideoEncoderNode',
                name='video_encoder',
                parameters=[
                    {'input_topic': '/image_raw'},
                    {'target_bitrate': target_bitrate_kbps},
                    {'x264_preset': x264_preset},
                    {'output_fps': 60},
                    {'packet_size': 300},
                    {'enable_display': False},
                    {'debug_dump_enable': debug_dump_enable},
                    {'debug_dump_every_n_frames': debug_dump_every_n_frames},
                    {'debug_dump_save_raw': dump_save_raw},
                    {'debug_dump_save_roi': dump_save_roi},
                    {'debug_dump_save_static': dump_save_static},
                    {'debug_dump_save_final': dump_save_final},
                    {'debug_dump_dir': debug_dump_dir},
                    {'crop_size': 800},
                    {'output_size': encode_size},
                    {'static_simplify': True},
                    {'motion_threshold': 14},
                    {'motion_erode_px': 2},
                    {'motion_dilate_px': 6},
                    {'motion_trail_frames': 90},
                    {'trail_disable_motion_ratio': 0.30},
                    {'bg_update_alpha': 0.01},
                    {'bg_blur_sigma': 1.8},
                    {'center_clear_size': 150},
                    {'force_monochrome': False},
                    {'bandwidth_limit_kbytes': hard_max_bitrate_kbytes},
                    {'bandwidth_window_s': 2.0},
                    {'max_tx_delay_s': 1.0}
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ],
        output='screen'
    )

    # 本地相机解码器 → 输出 /decoded_image/native
    decoder_local_node = Node(
        package='doorlock_decoder',
        executable='decoder_node',
        name='decoder_native_camera',
        parameters=[
            {'topic': '/video_stream'},
            {'format_topic': '/video_format'},
            {'display': enable_separate_decoder},
            {'default_codec': 'h264'},
            {'disable_auto_switch': True},
            {'window_title': '本地相机编码流'},
            {'publish_image': True},
            {'width': encode_size},
            {'height': encode_size},
            {'display_scale': 2},
            {'reset_on_gap': False},
            {'gap_reset_threshold': 20},
            {'decode_error_reset_threshold': 15},
            {'crosshair_offset_x': 0},
            {'crosshair_offset_y': 0},
            {'crosshair_width': 1},
            {'debug_dump_enable': debug_dump_enable},
            {'debug_dump_every_n_frames': debug_dump_every_n_frames},
            {'debug_dump_save_decoder': dump_save_decoder},
            {'debug_dump_dir': debug_dump_dir}
        ],
        remappings=[
            ('/decoded_image', '/decoded_image/native')
        ],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        DeclareLaunchArgument('enable_separate_decoder', default_value='False'),
        encoder_container,
        decoder_local_node
    ])
