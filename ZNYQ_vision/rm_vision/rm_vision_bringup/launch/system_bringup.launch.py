import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction

def generate_launch_description():
    # 声明可选参数
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # 获取各包的路径
    projectile_motion_pkg = get_package_share_directory('projectile_motion')
    vision_bringup_pkg = get_package_share_directory('rm_vision_bringup')

    projectile_motion_launch = os.path.join(
        projectile_motion_pkg, 'launch', 'projectile_motion_launch.py'
    )

    vision_bringup_launch = os.path.join(
        vision_bringup_pkg, 'launch', 'vision_bringup.launch.py'
    )


    # 包含两个 launch 文件
    projectile_motion_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(projectile_motion_launch),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    vision_bringup_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vision_bringup_launch),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )   
    # 弹道解算节点稍后启动
    delay_projectile = TimerAction(
        period=2.0,
        actions=[projectile_motion_include],
    )

    # 组合为一个 LaunchDescription
    ld = LaunchDescription()

    # 声明参数
    ld.add_action(declare_use_sim_time)

    # 添加两个系统的启动
    ld.add_action(vision_bringup_include)
    ld.add_action(delay_projectile)
    return ld
