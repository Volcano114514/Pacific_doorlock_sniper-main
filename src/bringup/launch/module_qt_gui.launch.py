from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    # 强制Qt使用X11渲染，解决Wayland下窗口不显示问题
    set_qt_platform = SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')

    qt_gui_node = Node(
        package='doorlock_decoder_qt',
        executable='video_switch_gui',
        name='qt_video_switch_gui',
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        set_qt_platform,
        qt_gui_node
    ])
