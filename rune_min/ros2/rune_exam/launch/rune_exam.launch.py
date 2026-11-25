# 一条命令起识别 + 解算两个节点
#
#   ros2 launch rune_exam rune_exam.launch.py
#
# 先启动仿真器（../RM_Energy_Hit_Simulator*/start_simulator.sh），再跑这个。

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
    ])
