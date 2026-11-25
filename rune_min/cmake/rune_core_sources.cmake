# rune_core 的源文件清单 —— 全仓库只有这一份。
#
# ROS 包（../ros2/rune_exam/CMakeLists.txt）include 本文件。
# 加一个 .cpp 只需要改这里。
#
# 包含方要先把 RUNE_CORE_ROOT 指向 rune_min/ 根目录，例如：
#   set(RUNE_CORE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")
#   include("${RUNE_CORE_ROOT}/cmake/rune_core_sources.cmake")

if(NOT DEFINED RUNE_CORE_ROOT)
  message(FATAL_ERROR "include rune_core_sources.cmake 之前必须先 set(RUNE_CORE_ROOT ...)")
endif()

set(RUNE_CORE_SOURCES
  # 图像预处理：去畸变 / 缩放 / 坐标换算（识别节点在用）
  ${RUNE_CORE_ROOT}/src/preprocess.cpp
  # 输入源：视频 / 图片 / 相机 -> cv::Mat（rune_source_node 用）
  ${RUNE_CORE_ROOT}/src/frame_source.cpp
  # 姿态时间序列 + /rm_energy/debug/labels 解析（云台节点用，可离线单测）
  ${RUNE_CORE_ROOT}/src/pose_source.cpp
  # 视觉与解算
  ${RUNE_CORE_ROOT}/src/blade_detector.cpp
  # 跟踪：身份跨帧保持（track-by-detection）+ 该打哪一块的策略
  ${RUNE_CORE_ROOT}/src/blade_tracker.cpp
  ${RUNE_CORE_ROOT}/src/track_selector.cpp
  ${RUNE_CORE_ROOT}/src/pnp.cpp
)

# 2026-11 删除：串口三层（serial / serial_protocol / serial_port）和 5 臂激活状态机
# （state_machine）。考核 2.1~2.5 全是「视频 / 仿真器话题」，没有任何实车接口。
#
# 2026-11 删除：桌面演示链（main / pipeline / ekf / solver / target_selector / types）。
# 考核只用 ROS 节点，那条链的功能（预处理开关、可视化、录制）已经搬进
# rune_recognition_node。
