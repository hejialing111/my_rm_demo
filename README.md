# rune_min — 能量机关亮靶识别 + PnP 解算 + 云台击打

## 项目结构

```
.
├── rune_min/                          # 核心算法库 + ROS2 节点
│   ├── CMakeLists.txt                 # 核心库 (OpenCV, C++17)
│   ├── include/rune_min/              # 头文件
│   ├── src/                           # 核心实现：blade_detector, pnp, track_selector ...
│   ├── ros2/rune_exam/                # ROS2 包
│   │   ├── msg/                       # 自定义消息 (Blade, BladeKeypoints, TargetInfo)
│   │   ├── config/rune_exam.yaml      # 所有节点参数
│   │   ├── launch/                    # rune_video / rune_hit / rune_exam
│   │   └── src/                       # recognition / solver / gimbal / source 节点
│   └── tools/debug_view.py            # 调试图查看窗口
├── RM_Energy_Hit_Simulator_Linux_20261009_RedR/   # Unity 仿真器 + ROS2 bridge
│   └── ros2_ws/                       # 仿真工作空间 (rm_energy_bridge, rm_energy_interfaces)
└── big_11.avi                         # 测试视频
```

### ROS2 节点

| 节点 | 功能 |
|---|---|
| `rune_source_node` | 视频源（相机 / 视频文件） |
| `rune_recognition_node` | 亮靶检测 — HSV 阈值 + 椭圆拟合 + 关键点提取 |
| `rune_solver_node` | PnP 解算 — 多点姿态估计 + 多切换靶跟踪 |
| `rune_gimbal_node` | 云台控制 — 弹道补偿 + 提前量 + 开火决策 |

---

## 环境依赖

- Ubuntu 22.04 + ROS 2 Humble
- OpenCV (`sudo apt install libopencv-dev`)
- Eigen 3 (`sudo apt install libeigen3-dev`)
- cv_bridge (`sudo apt install ros-humble-cv-bridge`)
- 调试图窗口：`python3 -c 'import cv2'`，没有就 `sudo apt install python3-opencv`

---

## 编译

### 1. 编译仿真器工作空间

```bash
source /opt/ros/humble/setup.bash
cd ~/demo_01/RM_Energy_Hit_Simulator_Linux_20261009_RedR/ros2_ws
colcon build --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
```

### 2. 编译 rune_exam

```bash
source /opt/ros/humble/setup.bash
source ~/demo_01/RM_Energy_Hit_Simulator_Linux_20261009_RedR/ros2_ws/install/setup.bash

mkdir -p ~/rune_ws/src
ln -sfn ~/demo_01/rune_min/ros2/rune_exam ~/rune_ws/src/rune_exam

cd ~/rune_ws
colcon build --packages-select rune_exam --symlink-install
source install/setup.bash
ros2 pkg executables rune_exam
```

> 只改 launch / yaml 也要重编（`colcon build` 拷贝到 `install/`）。改 `.msg` 必须重编并重启所有相关节点。

---

## 运行

### A. 视频识别（离线调试）

**终端 1** — 起视频源 + 识别：

```bash
source /opt/ros/humble/setup.bash
source ~/rune_ws/install/setup.bash

ros2 launch rune_exam rune_video.launch.py \
  video_path:=$HOME/demo_01/big_11.avi
```

**终端 2** — 开检测窗口：

```bash
source /opt/ros/humble/setup.bash
python3 ~/demo_01/rune_min/tools/debug_view.py
```

### B. 仿真击打

**终端 1** — 起仿真器：

```bash
cd ~/demo_01/RM_Energy_Hit_Simulator_Linux_20261009_RedR
chmod +x start_simulator.sh Simulator/RMEnergyHit.x86_64 2>/dev/null
./start_simulator.sh -screen-fullscreen 0 -screen-width 1280 -screen-height 720
```

**终端 2** — 识别 + 解算 + 云台：

```bash
source /opt/ros/humble/setup.bash
source ~/demo_01/RM_Energy_Hit_Simulator_Linux_20261009_RedR/ros2_ws/install/setup.bash
source ~/rune_ws/install/setup.bash

ros2 launch rune_exam rune_hit.launch.py
```

---

## 调试命令

```bash
source /opt/ros/humble/setup.bash
source ~/rune_ws/install/setup.bash

ros2 node list
ros2 topic hz /rune/blade_keypoints       # 约 30 Hz
ros2 topic echo /rune/blade_keypoints --once
ros2 topic echo /rune/target_info --once
ros2 param get /rune_recognition debug_fps
ros2 param get /rune_gimbal gimbal_lag_s
ros2 topic hz /rm_energy/gimbal/command    # 仿真模式
```

---

## 参数说明

全部参数在 `rune_min/ros2/rune_exam/config/rune_exam.yaml`，按节点分段。**参数只在节点启动时读一次，修改后必须重启节点。**

### rune_recognition（识别）

| 参数 | 作用 |
|---|---|
| `min_aspect` | 椭圆短轴/长轴下限（过严漏检，过松误检） |
| `min_ellipse_fill` | 椭圆面积填充率，剔不规则形状 |
| `min_solidity` | 面积/凸包下限，剔 R 标凹口 |
| `morph_px` | 闭运算核大小，固定 3 为主 |
| `debug_fps` / `debug_scale` | 调试图帧率 / 缩放（1.0 = 720×540） |

### rune_solver（解算）

| 参数 | 作用 |
|---|---|
| `feature_diameter_m` | PnP 尺度（0.308），填错距离按比例偏 |
| `switch_after_missed` | 连续丢几帧换靶 |
| `activate_confirm` | 连续几帧确认激活才换靶 |

### rune_gimbal（云台）

| 参数 | 作用 |
|---|---|
| `aim_kp` / `angle_filter_alpha` | 比例增益 / 平滑 |
| `gimbal_lag_s` | 云台一帧滞后 |
| `extra_latency_s` | 指令到云台到位剩余延迟，提前量补偿 |
| `fire_miss_m` | 开火误差门限 |

---

## 排错

| 现象 | 检查 |
|---|---|
| `setup.bash: 没有那个文件` | 仿真工作区路径不对或没编；`find ~ -name start_simulator.sh` |
| `executable not found` | 没重编，或 `rune_gimbal_node` 找不到 `rm_energy_interfaces` |
| 视频源报"打不开视频" | `video_path` 不对；`ls -l ~/demo_01/big_11.avi` |
| 识别日志空 | 视频源没发图，或 `compressed_topic` 对不上 |
| 调试图窗口没弹 | `python3 -c 'import cv2'` |
| 云台不开火 | 看 `miss` 是否进 `fire_miss_m` 门限 |
| 改了参数没生效 | 参数只在启动时读；重启节点 |