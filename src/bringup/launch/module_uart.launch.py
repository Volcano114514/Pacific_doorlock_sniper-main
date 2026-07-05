from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    declare_uart_port = DeclareLaunchArgument('uart_port', default_value='/dev/ttyACM0')
    declare_uart_baudrate = DeclareLaunchArgument('uart_baudrate', default_value='921600')
    declare_enable_receive = DeclareLaunchArgument('enable_receive', default_value='False')
    declare_enable_send = DeclareLaunchArgument('enable_send', default_value='False')

    uart_port = LaunchConfiguration('uart_port')
    uart_baudrate = LaunchConfiguration('uart_baudrate')
    enable_receive = LaunchConfiguration('enable_receive')
    enable_send = LaunchConfiguration('enable_send')

    # 全局唯一串口收发一体节点
    uart_transceiver_node = Node(
        package='doorlock_sniper',
        executable='video_uart_transceiver',
        name='video_uart_transceiver',
        parameters=[
            {'uart_device': uart_port},
            {'baudrate': uart_baudrate},
            {'enable_send': enable_send},
            {'enable_receive': enable_receive},
            {'debug_mode': enable_receive}
        ],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        declare_uart_port,
        declare_uart_baudrate,
        declare_enable_receive,
        declare_enable_send,
        uart_transceiver_node
    ])
