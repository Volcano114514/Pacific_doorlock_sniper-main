from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, LaunchConfigurationEquals
from pathlib import Path


def generate_launch_description():
    launch_dir = Path(__file__).resolve().parent

    # 全局参数
    declare_uart_enable = DeclareLaunchArgument('uart_enable', default_value='False')
    declare_uart_port = DeclareLaunchArgument('uart_port', default_value='/dev/ttyACM0')
    declare_uart_baud = DeclareLaunchArgument('uart_baudrate', default_value='921600')
    declare_use_camera = DeclareLaunchArgument('use_camera', default_value='False')
    declare_debug_mode = DeclareLaunchArgument('debug_mode', default_value='False')
    declare_separate_decoder = DeclareLaunchArgument('enable_separate_decoder', default_value='False')
    declare_use_shark = DeclareLaunchArgument('use_shark', default_value='False')
    declare_shark_ip = DeclareLaunchArgument('shark_ip', default_value='192.168.12.1')
    declare_shark_port = DeclareLaunchArgument('shark_port', default_value='3334')
    declare_shark_timeout = DeclareLaunchArgument('shark_frame_timeout_s', default_value='1.0')
    declare_use_mqtt = DeclareLaunchArgument('use_mqtt', default_value='False')
    declare_mqtt_ip = DeclareLaunchArgument('mqtt_server_ip', default_value='192.168.1')
    declare_mqtt_port = DeclareLaunchArgument('mqtt_server_port', default_value='3333')
    declare_robot_id = DeclareLaunchArgument('robot_id', default_value='102')
    declare_use_qt = DeclareLaunchArgument('use_qt_gui', default_value='True')

    # 相机+编码
    encoder_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / 'module_encoder_camera.launch.py')),
        launch_arguments={'enable_separate_decoder': LaunchConfiguration('enable_separate_decoder')}.items(),
        condition=LaunchConfigurationEquals('use_camera', 'True')
    )

    # Shark UDP图传核心
    shark_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / 'module_shark.launch.py')),
        launch_arguments={
            'sh_ip': LaunchConfiguration('shark_ip'),
            'sh_port': LaunchConfiguration('shark_port'),
            'shark_frame_timeout_s': LaunchConfiguration('shark_frame_timeout_s'),
            'enable_separate_decoder': LaunchConfiguration('enable_separate_decoder')
        }.items(),
        condition=LaunchConfigurationEquals('use_shark', 'True')
    )

    # MQTT
    mqtt_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / 'module_mqtt.launch.py')),
        launch_arguments={
            'mqtt_server_ip': LaunchConfiguration('mqtt_server_ip'),
            'mqtt_server_port': LaunchConfiguration('mqtt_server_port'),
            'robot_id': LaunchConfiguration('robot_id'),
            'enable_separate_decoder': LaunchConfiguration('enable_separate_decoder'),
            'enable_video_stream': PythonExpression(["'False' if '", LaunchConfiguration('debug_mode'), "' == 'True' else 'True'"])
        }.items(),
        condition=LaunchConfigurationEquals('use_mqtt', 'True')
    )

    # 串口收发（UDP无关，保留）
    uart_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / 'module_uart.launch.py')),
        launch_arguments={
            'uart_port': LaunchConfiguration('uart_port'),
            'uart_baudrate': LaunchConfiguration('uart_baudrate'),
            'enable_receive': LaunchConfiguration('debug_mode'),
            'enable_send': PythonExpression(["'True' if '", LaunchConfiguration('use_camera'), "' == 'True' else 'False'"])
        }.items(),
        condition=LaunchConfigurationEquals('uart_enable', 'True')
    )

    # 串口解码器
    uart_decoder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / 'module_uart_decoder.launch.py')),
        launch_arguments={'enable_separate_decoder': LaunchConfiguration('enable_separate_decoder')}.items(),
        condition=LaunchConfigurationEquals('debug_mode', 'True')
    )

    # QT界面
    qt_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / 'module_qt_gui.launch.py')),
        condition=LaunchConfigurationEquals('use_qt', 'True')
    )

    return LaunchDescription([
        declare_uart_enable, declare_uart_port, declare_uart_baud,
        declare_use_camera, declare_debug_mode, declare_separate_decoder,
        declare_use_shark, declare_shark_ip, declare_shark_port, declare_shark_timeout,
        declare_use_mqtt, declare_mqtt_ip, declare_mqtt_port, declare_robot_id,
        declare_use_qt,
        encoder_camera_launch,
        shark_launch,
        mqtt_launch,
        uart_launch,
        uart_decoder_launch,
        qt_launch
    ])
