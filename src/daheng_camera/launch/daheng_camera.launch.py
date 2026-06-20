from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition

def generate_launch_description():
    # 声明可配置参数
    camera_name_arg = DeclareLaunchArgument(
        'camera_name', 
        default_value='daheng_camera',
        description='相机节点名称'
    )
    camera_info_url_arg = DeclareLaunchArgument(
        'camera_info_url', 
        default_value='/home/vision/Pacific_doorlock_sniper-main/src/daheng_camera/config/camera_6mm_MER2-160-227U3M.yaml.yaml',
        description='相机标定文件路径（留空则使用默认内参）'
    )
    exposure_time_arg = DeclareLaunchArgument(
        'exposure_time', 
        default_value='10000.0',
        description='曝光时间（微秒）'
    )
    gain_arg = DeclareLaunchArgument(
        'gain', 
        default_value='0.0',
        description='增益（dB）'
    )
    use_sensor_qos_arg = DeclareLaunchArgument(
        'use_sensor_data_qos', 
        default_value='true',
        description='是否使用传感器数据QoS（低延迟）'
    )
    log_level_arg = DeclareLaunchArgument(
        'log_level', 
        default_value='info',
        description='日志级别：debug/info/warn/error/fatal'
    )
    # 启动图像查看器开关（默认不启动）
    launch_viewer_arg = DeclareLaunchArgument(
        'launch_viewer', 
        default_value='true',
        description='是否同时启动rqt_image_view查看图像'
    )

    # 相机节点
    daheng_camera_node = Node(
        package='daheng_camera',
        executable='daheng_camera_node',
        name=LaunchConfiguration('camera_name'),
        output='screen',
        parameters=[{
            'camera_name': LaunchConfiguration('camera_name'),
            'camera_info_url': LaunchConfiguration('camera_info_url'),
            'use_sensor_data_qos': LaunchConfiguration('use_sensor_data_qos'),
            'exposure_time': LaunchConfiguration('exposure_time'),
            'gain': LaunchConfiguration('gain'),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        # 仅保留必要的话题映射
        remappings=[
            ('image_raw', 'image_raw'),
            ('camera_info', 'camera_info'),
        ],
    )

    # 图像查看器（需要安装 ros-humble-rqt-image-view）
    image_viewer_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='camera_viewer',
        arguments=['/daheng_camera/image_raw'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_viewer'))
    )

    return LaunchDescription([
        camera_name_arg,
        camera_info_url_arg,
        exposure_time_arg,
        gain_arg,
        use_sensor_qos_arg,
        log_level_arg,
        launch_viewer_arg,
        daheng_camera_node,
        image_viewer_node,
    ])