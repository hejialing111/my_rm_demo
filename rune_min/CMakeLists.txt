cmake_minimum_required(VERSION 3.16)
project(rune_min LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)

# 源文件清单由 cmake/rune_core_sources.cmake 统一维护，ROS 包也 include 同一份。
set(RUNE_CORE_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
include(${RUNE_CORE_ROOT}/cmake/rune_core_sources.cmake)

# 核心算法库。真正的可执行程序在 ros2/rune_exam（识别 / 解算 / 云台节点）。
add_library(rune_core ${RUNE_CORE_SOURCES})

target_include_directories(rune_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(rune_core PUBLIC ${OpenCV_LIBS})

if (MSVC)
    target_compile_options(rune_core PRIVATE /W4)
else()
    target_compile_options(rune_core PRIVATE -Wall -Wextra)
endif()
