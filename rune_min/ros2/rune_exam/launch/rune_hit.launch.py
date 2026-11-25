# 2.5：识别 + 解算 + 云台击打，一条命令起三个节点
#
#   ros2 launch rune_exam rune_hit.launch.py
#
# 需要仿真器提供的 rm_energy_interfaces（先 source 它的 ros2_ws）。
# 只过 2.1~2.4 时，单独起 rune_recognition_node / rune_solver_node（不带云台）。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('rune_exam'), 'config', 'rune_exam.yaml')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='参数文件，默认用包内的 config/rune_exam.yaml'),

        Node(
            package='rune_exam', executable='rune_recognition_node',
            name='rune_recognition', output='screen',
            parameters=[params_file],
        ),

        Node(
            package='rune_exam', executable='rune_solver_node',
            name='rune_solver', output='screen',
            parameters=[params_file],
        ),

        Node(
            package='rune_exam', executable='rune_gimbal_node',
            name='rune_gimbal', output='screen',
            parameters=[params_file],
        ),
    ])
