import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 获取你的配置文件路径
    pkg_share = get_package_share_directory('mcnm_robot_cartographer')
    cartographer_config_dir = os.path.join(pkg_share, 'config')
    cartographer_config_basename = 'cartographer_2d.lua'

    # 1. 启动 Cartographer 核心节点
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}], # 如果跑仿真设为 True，实车设为 False
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', cartographer_config_basename],
        # 如果你的小车雷达话题不是 /scan，里程计不是 /odom，需要在这里重映射
        remappings=[
            ('/scan', '/scan'),
            ('/odom', '/odom')
        ]
    )

    # 2. 启动占据栅格地图发布节点 (负责把 Submap 转化为 /map 话题)
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    )

    return LaunchDescription([
        cartographer_node,
        occupancy_grid_node,
    ])