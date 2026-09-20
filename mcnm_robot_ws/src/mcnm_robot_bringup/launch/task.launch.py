import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('mcnm_robot_bringup')
    cfg = os.path.join(pkg, 'config', 'task_points.yaml')

    task_executor_node = Node(
        package='mcnm_robot_bringup',
        executable='task_executor',
        name='task_executor',
        output='screen',
        parameters=[cfg],
    )

    return LaunchDescription([task_executor_node])