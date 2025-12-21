import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包路径
    package_dir = get_package_share_directory('yolo_detect')
    
    # 配置文件路径
    config_file = os.path.join(package_dir, 'config', 'sentry.yaml')
    
    # 声明launch参数
    config_arg = DeclareLaunchArgument(
        name='config_file',
        default_value=config_file,
        description='自动瞄准检测器配置文件路径'
    )
    
    debug_arg = DeclareLaunchArgument(
        name='debug_mode',
        default_value='true',
        description='是否开启调试模式'
    )
    
    image_topic_arg = DeclareLaunchArgument(
        name='image_topic',
        default_value='/image_raw',
        description='订阅的图像话题名称'
    )
    
    # 创建检测节点
    detector_node = Node(
        package='yolo_detect',
        executable='yolo_detect_node',
        name='yolo_detect_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'config_path': LaunchConfiguration('config_file'),
            'debug_mode': LaunchConfiguration('debug_mode'),
            'image_topic': LaunchConfiguration('image_topic'),
        }]
    )
    
    return LaunchDescription([
        config_arg,
        debug_arg,
        image_topic_arg,
        detector_node
    ])