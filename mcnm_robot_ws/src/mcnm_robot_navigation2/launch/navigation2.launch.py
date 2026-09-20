import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 路径配置
    pkg_nav = get_package_share_directory('mcnm_robot_navigation2')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    # 默认参数文件路径
    default_map = os.path.join(pkg_nav, 'room', 'qizhen_room.yaml')
    default_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    default_rviz = os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')
    # 启动参数（命令行可覆盖）
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map yaml file'
    )

    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to the ROS2 parameters file for nav2 nodes'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    # 包含 Nav2官方bringup
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': LaunchConfiguration('autostart'),
        }.items()
    )

    # RViz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'rviz_launch.py')
        ),
        launch_arguments={
            'rviz_config': default_rviz,
        }.items(),
        condition=__import__('launch.conditions', fromlist=['IfCondition']).IfCondition(
            LaunchConfiguration('use_rviz')
        )
    )

    return LaunchDescription([
        declare_map,
        declare_params,
        declare_use_sim_time,
        declare_autostart,
        declare_use_rviz,
        nav2_bringup_launch,
        rviz_launch,
    ])