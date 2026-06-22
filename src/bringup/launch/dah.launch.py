from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from pathlib import Path


def generate_launch_description():
    launch_path = Path(__file__).resolve()
    project_root = launch_path.parents[3]
    if project_root.name == 'bringup' and (project_root / 'share').exists():
        project_root = project_root.parents[1]
    
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
    encode_size = 300  # 你的实际输出尺寸300x300

    # 海康相机参数
    wb_auto = 'Off'
    wb_red = 1.7383
    wb_blue = 2.0
    
    uart_enable = LaunchConfiguration('uart_enable')
    uart_port = LaunchConfiguration('uart_port')
    uart_baudrate = LaunchConfiguration('uart_baudrate')
    
    use_shark = LaunchConfiguration('use_shark')
    shark_ip = LaunchConfiguration('shark_ip')
    shark_port = LaunchConfiguration('shark_port')
    shark_frame_timeout_s = LaunchConfiguration('shark_frame_timeout_s')

    # 声明启动参数（仅使用Humble最基础的语法）
    declare_uart_enable_arg = DeclareLaunchArgument(
        'uart_enable',
        default_value='True',
        description='是否启用UART发送到STM32 (True/False)'
    )
    
    declare_uart_port_arg = DeclareLaunchArgument(
        'uart_port',
        default_value='/dev/ttyACM0',
        description='串口设备路径'
    )
    
    declare_uart_baudrate_arg = DeclareLaunchArgument(
        'uart_baudrate',
        default_value='921600',
        description='串口波特率'
    )
    
    declare_use_shark_arg = DeclareLaunchArgument(
        'use_shark',
        default_value='True',
        description='是否使用SharkDataServer远程模拟源 (True/False)'
    )
    
    declare_shark_ip_arg = DeclareLaunchArgument(
        'shark_ip',
        default_value='172.20.10.3',
        description='SharkDataServer服务器IP地址'
    )
    
    declare_shark_port_arg = DeclareLaunchArgument(
        'shark_port',
        default_value='3334',
        description='SharkDataServer UDP视频流端口'
    )
    
    declare_shark_frame_timeout_arg = DeclareLaunchArgument(
        'shark_frame_timeout_s',
        default_value='1.0',
        description='Shark视频帧超时时间'
    )

    # 本地相机+编码器容器 - 仅在use_shark=False时启动
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
                    {'enable_display': True},
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
        output='screen',
        condition=LaunchConfigurationNotEquals('use_shark', 'True')
    )

    # 解码节点 - 两种模式都启动
    decoder_node = Node(
        package='doorlock_decoder',
        executable='decoder_node',
        name='video_decoder',
        parameters=[
            {'topic': '/video_stream'},
            {'display': True},
            {'width': encode_size},
            {'height': encode_size},
            {'display_scale': 2},
            {'crosshair_offset_x': 0},
            {'crosshair_offset_y': 0},
            {'crosshair_width': 1},
            {'debug_dump_enable': debug_dump_enable},
            {'debug_dump_every_n_frames': debug_dump_every_n_frames},
            {'debug_dump_save_decoder': dump_save_decoder},
            {'debug_dump_dir': debug_dump_dir}
        ],
        output='screen',
        emulate_tty=True,
    )

    # UART发送节点 - 仅在uart_enable=True时启动
    # 注意：Shark模式下请手动加上uart_enable:=False避免启动无用进程
    uart_sender_node = Node(
        package='doorlock_sniper',
        executable='video_uart_sender',
        name='video_uart_sender',
        parameters=[
            {'serial_port': uart_port},
            {'baud_rate': uart_baudrate}
        ],
        output='screen',
        emulate_tty=True,
        condition=LaunchConfigurationEquals('uart_enable', 'True')
    )
    
    # Shark桥接节点 - 仅在use_shark=True时启动
    shark_bridge_node = Node(
        package='doorlock_decoder',
        executable='shark_bridge_node',
        name='shark_bridge',
        parameters=[
            {'shark_ip': shark_ip},
            {'shark_port': shark_port},
            {'frame_timeout_s': shark_frame_timeout_s},
            {'packet_size': 300}
        ],
        output='screen',
        emulate_tty=True,
        condition=LaunchConfigurationEquals('use_shark', 'True')
    )

    return LaunchDescription([
        declare_uart_enable_arg,
        declare_uart_port_arg,
        declare_uart_baudrate_arg,
        declare_use_shark_arg,
        declare_shark_ip_arg,
        declare_shark_port_arg,
        declare_shark_frame_timeout_arg,
        encoder_container,
        decoder_node,
        uart_sender_node,
        shark_bridge_node
    ])
