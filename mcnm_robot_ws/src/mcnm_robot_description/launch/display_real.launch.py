import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('mcnm_robot_description')

    xacro_file = os.path.join(
        pkg_share, 'urdf', 'mcnm_robot', 'mcnm_robot_assembly.urdf.xacro'
    )
    rviz_config = os.path.join(pkg_share, 'config', 'display.rviz')
    laser_filter_config = os.path.join(pkg_share, 'config', 'laser_filter.yaml')

    rplidar_share = get_package_share_directory('rplidar_ros')
    rplidar_launch_file = os.path.join(
        rplidar_share, 'launch', 'rplidar_c1_launch.py'
    )

    # ==================== 启动参数 ====================
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to start the model-display RViz'
    )

    # ==================== URDF ====================
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # ==================== 节点 ====================
    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['udp4', '--port', '8888']
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    odom2tf_node = Node(
        package='mcnm_robot_bringup',
        executable='odom2tf',
        name='odom2tf',
        output='screen'
    )

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch_file)
    )

    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_filter',
        output='screen',
        parameters=[laser_filter_config],
        remappings=[
            ('scan', '/scan'),
            ('scan_filtered', '/scan_filtered'),
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz'))   # ← 受参数控制
    )

    cooling_fan_node = Node(
        package='cooling_fan_controller',
        executable='cooling_fan_control',
        name='cooling_fan_controller',
        output='screen'
    )

    return LaunchDescription([
        declare_use_rviz,
        micro_ros_agent_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        odom2tf_node,
        rplidar_launch,
        laser_filter_node,
        rviz_node,
        cooling_fan_node,
    ])