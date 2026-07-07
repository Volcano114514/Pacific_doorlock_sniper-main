from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ComposableNodeContainer, LoadComposableNodes
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    declare_separate_decoder = DeclareLaunchArgument('enable_separate_decoder', default_value='False')
    sep_decoder = LaunchConfiguration('enable_separate_decoder')

    container = ComposableNodeContainer(
        name='sniper_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen'
    )

    # 大恒相机
    camera_node = ComposableNode(
        package='daheng_camera',
        plugin='daheng_camera::DahengCameraNode',
        name='daheng_camera'
    )

    # H264编码器
    encoder_node = ComposableNode(
        package='doorlock_sniper',
        plugin='doorlock_sniper::VideoEncoderNode',
        name='video_encoder'
    )

    load_nodes = LoadComposableNodes(
        target_container='sniper_container',
        composable_node_descriptions=[camera_node, encoder_node]
    )

    # 本地解码器（本地预览，和UDP无关）
    local_decoder = Node(
        package='doorlock_decoder',
        executable='decoder_node',
        name='decoder_native_camera',
        parameters=[
            {'topic': '/video_stream'},
            {'use_variable_packet': False},
            {'display': sep_decoder},
            {'default_codec': 'h264'},
            {'window_title': '本地相机画面'}
        ],
        remappings=[('/decoded_image', '/decoded_image/native')],
        output='screen'
    )

    return LaunchDescription([
        declare_separate_decoder,
        container,
        load_nodes,
        local_decoder
    ])
