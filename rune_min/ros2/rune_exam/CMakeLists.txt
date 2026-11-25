cmake_minimum_required(VERSION 3.16)
project(rune_exam)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
# 自定义消息必需：rosidl_generate_interfaces 这个宏由它提供。
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# 2.5（云台击打）需要仿真器自带的接口包；没有它也能编识别/解算两个节点。
find_package(rm_energy_interfaces QUIET)

# ---------------------------------------------------------------- 接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Blade.msg"
  "msg/BladeKeypoints.msg"
  "msg/TargetInfo.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# ---------------------------------------------------------------- 纯 C++ 核心
# 复用桌面 demo 的同一份 rune_core，源文件清单只维护一份。
#
# 注意：包一般是「符号链接」挂进 workspace 的（ws/src/rune_exam -> 仓库/ros2/rune_exam）。
# 这时 CMAKE_CURRENT_SOURCE_DIR 是链接路径，直接 ../.. 会指到 workspace 而不是仓库，
# 所以先 REALPATH 把链接解开、再上溯两级。
if(NOT DEFINED RUNE_CORE_ROOT)
  get_filename_component(_rune_pkg_dir "${CMAKE_CURRENT_SOURCE_DIR}" REALPATH)
  get_filename_component(RUNE_CORE_ROOT "${_rune_pkg_dir}/../.." REALPATH)
endif()

if(NOT EXISTS "${RUNE_CORE_ROOT}/include/rune_min/config.hpp")
  message(FATAL_ERROR
    "找不到 rune_core（当前解析到 ${RUNE_CORE_ROOT}）。\n"
    "如果包被复制到了别处，请显式指定根目录：\n"
    "  colcon build --packages-select rune_exam --cmake-args -DRUNE_CORE_ROOT=/path/to/rune_min")
endif()

include("${RUNE_CORE_ROOT}/cmake/rune_core_sources.cmake")

add_library(rune_core STATIC ${RUNE_CORE_SOURCES})
target_include_directories(rune_core PUBLIC
  ${RUNE_CORE_ROOT}/include
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rune_core PUBLIC ${OpenCV_LIBS})

# ---------------------------------------------------------------- 节点
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 识别节点：图像 -> 亮靶关键点（考核 2.2 / 2.3）
add_executable(rune_recognition_node src/recognition_node.cpp)
ament_target_dependencies(rune_recognition_node
  rclcpp sensor_msgs geometry_msgs std_msgs)
target_link_libraries(rune_recognition_node rune_core "${cpp_typesupport_target}")

# 输入源节点：考核 2.2 的视频回放（视频/图片/相机 -> CompressedImage）
# 让识别节点改订 /rune/image_raw/compressed 即可，检测逻辑完全不变。
add_executable(rune_source_node src/rune_source_node.cpp)
ament_target_dependencies(rune_source_node
  rclcpp sensor_msgs std_msgs cv_bridge)
target_link_libraries(rune_source_node rune_core "${cpp_typesupport_target}")

# 解算节点：关键点 -> 图像坐标 + 距离（考核 2.4）
add_executable(rune_solver_node src/solver_node.cpp)
ament_target_dependencies(rune_solver_node
  rclcpp sensor_msgs geometry_msgs std_msgs)
target_link_libraries(rune_solver_node rune_core "${cpp_typesupport_target}")

# 云台节点：目标 -> 预测 + 弹道 + yaw/pitch/fire（考核 2.5）
if(rm_energy_interfaces_FOUND)
  add_executable(rune_gimbal_node src/gimbal_node.cpp)
  ament_target_dependencies(rune_gimbal_node
    rclcpp std_msgs geometry_msgs rm_energy_interfaces)
  target_link_libraries(rune_gimbal_node rune_core Eigen3::Eigen "${cpp_typesupport_target}")
  install(TARGETS rune_gimbal_node DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS
    "rm_energy_interfaces 未找到 -> 跳过 rune_gimbal_node（2.5 需要它；"
    "先 source 仿真器的 ros2_ws/install/setup.bash）")
endif()

install(TARGETS
    rune_recognition_node
    rune_solver_node
    rune_source_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
