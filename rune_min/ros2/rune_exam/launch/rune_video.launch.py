# 视频回放（考核 2.2）：视频源 + 识别节点，识别节点同时发 debug 图。
#
#   ros2 launch rune_exam rune_video.launch.py video_path:=/abs/path/big_11.avi
#   ros2 run rqt_image_view rqt_image_view /rune/debug/image/compressed
#
# debug 图上：绿=靶椭圆/中心/编号，红=5 个关键点，蓝=机关圆心，黄=相位方向。

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
    video_path = LaunchConfiguration('video_path')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='参数文件，默认用包内的 config/rune_exam.yaml'),
        DeclareLaunchArgument(
            'video_path', description='big_11.avi 等视频的绝对路径'),

        Node(
            package='rune_exam', executable='rune_source_node',
            name='rune_source', output='screen',
            parameters=[{
                'source_type': 'video',
                'path': video_path,
                'loop': True,
                'topic': '/rune/image_raw',
                'format': 'jpeg',
                # 不缩图：保证识别输出的是视频原图坐标（例如 1440x1080）。
                'scale': 1.0,
            }],
        ),

        Node(
            package='rune_exam', executable='rune_recognition_node',
            name='rune_recognition', output='screen',
            parameters=[params_file, {
                'compressed_topic': '/rune/image_raw/compressed',
                'publish_debug_image': True,
                # 检测在 0.5 图上跑，输出坐标会换算回视频原图。
                'detect_scale': 0.5,
                # 调试图只给人看：窗口里 720x540，不影响话题坐标。
                'debug_fps': 15.0,
                'debug_scale': 0.5,

                # ---- 视频场景专用：压误检 ----
                # 这段视频（big_11.avi）是**蓝方**灯效：实测靶面 H≈114 S≈245 V≈150~180，
                # 红色区域只有零散反光、凑不出椭圆（red 单独跑 300 帧 0 候选）。
                # 所以这里必须用 blue；仿真器那边是红灯，走 yaml 里的 both，互不影响。
                'rune_color': 'blue',
                # 关掉形状兜底：一帧里没有合格候选时不再"硬发一个形状不达标的"。
                # 这一条直接消掉"根本不是椭圆、轮廓不闭合也被检测到"。
                'allow_shape_fallback': False,
                # 视频里的靶面半径约 44 px（检测图 0.5 倍 → ~22 px），面积 ~1500 px²。
                # 原来 0.0002 在 720x540 上只有 78 px²，一个 10x8 的亮斑就过 —— 这就是
                # "很小的实体区域被识别"的来源。抬到 0.0012（466 px²）后，小碎块全没了。
                'min_area_ratio': 0.0012,
                'min_radius_px': 8.0,
                # 椭圆判据收紧一点：实测真靶 ellipse_fill 0.97~1.00、solidity 0.97。
                'min_ellipse_fill': 0.88,
                'min_solidity': 0.92,
            }],
        ),
    ])
