from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    set_qt_env = SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')
    declare_shark_ip = DeclareLaunchArgument(
        'shark_ip', default_value='192.168.12.1',
        description='Shark UDP server IP address')
    declare_shark_port = DeclareLaunchArgument(
        'shark_port', default_value='3334',
        description='Shark UDP server port')
    declare_mqtt_ip = DeclareLaunchArgument(
        'mqtt_ip', default_value='192.168.12.1',
        description='MQTT broker IP address')
    declare_mqtt_port = DeclareLaunchArgument(
        'mqtt_port', default_value='3333',
        description='MQTT broker port')
    declare_client_id = DeclareLaunchArgument(
        'client_id', default_value='101',
        description='Robot ID for MQTT client (see protocol appendix for IDs)')
    declare_codec = DeclareLaunchArgument(
        'default_codec', default_value='hevc',
        description='Video codec: hevc or h264')

    shark_ip_cfg = LaunchConfiguration('shark_ip')
    shark_port_cfg = LaunchConfiguration('shark_port')
    mqtt_ip_cfg = LaunchConfiguration('mqtt_ip')
    mqtt_port_cfg = LaunchConfiguration('mqtt_port')
    client_id_cfg = LaunchConfiguration('client_id')
    codec_cfg = LaunchConfiguration('default_codec')

    shark_bridge = Node(
        package='doorlock_decoder',
        executable='shark_bridge_node',
        name='shark_bridge',
        parameters=[{
            'shark_ip': shark_ip_cfg,
            'shark_port': shark_port_cfg,
            'packet_size': 65535
        }],
        output='screen'
    )
    shark_decoder = Node(
        package='doorlock_decoder',
        executable='decoder_node',
        name='shark_decoder',
        parameters=[
            {'topic': '/video_stream/shark'},
            {'output_topic': '/decoded_image/shark'},
            {'use_variable_packet': True},
            {'display': False},
            {'publish_image': True},
            {'default_codec': codec_cfg},
            {'disable_auto_switch': True}
        ],
        output='screen'
    )
    # 核心修复：用PythonExpression强制client_id转为字符串，避免int传入
    custom_client = Node(
        package='doorlock_decoder',
        executable='custom_client_node',
        name='custom_client',
        parameters=[{
            'mqtt_broker_ip': mqtt_ip_cfg,
            'mqtt_broker_port': mqtt_port_cfg,
            'client_id': PythonExpression(['"', client_id_cfg, '"'])
        }],
        output='screen'
    )
    custom_decoder = Node(
        package='doorlock_decoder',
        executable='decoder_node',
        name='custom_video_decoder',
        parameters=[
            {'topic': '/custom_video_stream_packet'},
            {'output_topic': '/decoded_image/custom'},
            {'use_variable_packet': True},
            {'display': False},
            {'publish_image': True},
            {'default_codec': codec_cfg},
            {'disable_auto_switch': True}
        ],
        output='screen'
    )
    qt_gui = Node(
        package='doorlock_decoder_qt',
        executable='video_switch_gui',
        name='qt_video_switch',
        output='screen',
        emulate_tty=True
    )
    return LaunchDescription([
        set_qt_env,
        declare_shark_ip,
        declare_shark_port,
        declare_mqtt_ip,
        declare_mqtt_port,
        declare_client_id,
        declare_codec,
        shark_bridge,
        shark_decoder,
        custom_client,
        custom_decoder,
        qt_gui,
    ])
