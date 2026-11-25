# rune_min — 演示运行手册
---
## 1. 编译仿真器工作空间

```bash
source /opt/ros/humble/setup.bash
cd ~/demo_01/RM_Energy_Hit_Simulator_Linux_20261009_RedR/ros2_ws
colcon build --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
```
## 2. 编译 rune_exam

```bash
source /opt/ros/humble/setup.bash

mkdir -p ~/rune_ws/src
ln -sfn ~/demo_01/demo_01/rune_min/ros2/rune_exam ~/rune_ws/src/rune_exam

cd ~/rune_ws
colcon build --packages-select rune_exam
source install/setup.bash
ros2 pkg executables rune_exam
```

```bash
source /opt/ros/humble/setup.bash
source ~/demo_01/RM_Energy_Hit_Simulator_Linux_20261009_RedR/ros2_ws/install/setup.bash

mkdir -p ~/rune_ws/src
ln -sfn ~/demo_01/demo_01/rune_min/ros2/rune_exam ~/rune_ws/src/rune_exam

cd ~/rune_ws
colcon build --packages-select rune_exam
source install/setup.bash
ros2 pkg executables rune_exam
```
## 3. 依赖

```bash
# 视频源节点的 raw 分支
sudo apt install ros-humble-cv-bridge

# 调试图播放窗口
python3 -c 'import cv2' || sudo apt install python3-opencv
```

# 视频识别

## 第 1 步：终端 1 —— 起视频源 + 识别

```bash
source /opt/ros/humble/setup.bash
source ~/rune_ws/install/setup.bash

ros2 launch rune_exam rune_video.launch.py \
  video_path:=$HOME/demo_01/big_11.avi
`
## 第 2 步：终端 2 —— 开检测窗口

```bash
source /opt/ros/humble/setup.bash
python3 ~/demo_01/rune_min/tools/debug_view.py
```

---

# 仿真击打（2.5）

## 第 1 步：终端 1 —— 仿真器

```bash
cd ~/demo_01/RM_Energy_Hit_Simulator_Linux_20261009_RedR
# 从 Windows 同步后执行位可能丢，补一次
chmod +x start_simulator.sh Simulator/RMEnergyHit.x86_64 2>/dev/null
./start_simulator.sh -screen-fullscreen 0 -screen-width 1280 -screen-height 720
```
```
## 第 2 步：终端 2 —— 识别 + 解算 + 云台
cd /home/hejialing/rune_ws
colcon build --packages-select rune_exam --symlink-install
source install/setup.bash
ros2 launch rune_exam rune_hit.launch.py

```bash
source /opt/ros/humble/setup.bash
source ~/demo_01/RM_Energy_Hit_Simulator_Linux_20261009_RedR/ros2_ws/install/setup.bash
source ~/rune_ws/install/setup.bash

ros2 launch rune_exam rune_hit.launch.py \
  params_file:=$HOME/demo_01/demo_01/rune_min/ros2/rune_exam/config/rune_exam.yaml
``

## 结束
source /opt/ros/humble/setup.bash
source ~/demo_01/RM_Energy_Hit_Simulator_Linux_20261009_RedR/ros2_ws/install/setup.bash
cd ~/rune_ws
colcon build --packages-select rune_exam
source install/setup.bash
```
---

# 附 C：自检命令

```bash
source /opt/ros/humble/setup.bash
source ~/rune_ws/install/setup.bash

ros2 node list
ros2 pkg executables rune_exam

ros2 topic hz /rune/blade_keypoints          # 约 30 Hz
ros2 topic echo /rune/blade_keypoints --once
ros2 topic echo /rune/target_info --once

ros2 param get /rune_recognition debug_fps
ros2 param get /rune_gimbal gimbal_lag_s

ros2 topic hz /rm_energy/gimbal/command      # 仿真 2.5
```

---

# 附 D：参数

全在 `config/rune_exam.yaml`，按节点分段。最常动的：

| 节点 | 参数 | 作用 |
|---|---|---|
| 识别 | `min_aspect` / `min_ellipse_fill` / `min_solidity` | 椭圆三判据，误检多就收紧 |
| 识别 | `morph_px` | 闭运算核，固定 3 为主 |
| 识别 | `publish_debug_image` / `debug_fps` / `debug_scale` | 调试图开关/帧率/缩放 |
| 解算 | `feature_diameter_m` | PnP 尺度，0.308；填错距离按比例偏 |
| 解算 | `switch_after_missed` / `activate_confirm` | 换靶/激活确认 |
| 云台 | `aim_kp` / `angle_filter_alpha` | 比例增益/平滑 |
| 云台 | `fire_miss_m` / `fire_settle_s` | 开火门限/换靶后等待 |
| 云台 | `gimbal_lag_s` / `extra_latency_s` | 云台一帧滞后/额外提前量 |

参数**只在节点启动时读一次**，改完必须重启；用 `ros2 param get` 确认。

---

# 附 E：改了代码后重编

```bash
source /opt/ros/humble/setup.bash
source ~/demo_01/RM_Energy_Hit_Simulator_Linux_20261009_RedR/ros2_ws/install/setup.bash
cd ~/rune_ws
colcon build --packages-select rune_exam
source install/setup.bash
```

- 只改 launch / yaml 也要重编（`install/` 里是拷贝），或直接改 `install/rune_exam/share/rune_exam/...` 下那份；
- 改了 `.msg`：必须重编，并重启所有相关节点。

---

# 附 F：排错

| 现象 | 检查 |
|---|---|
| `source .../ros2_ws/install/setup.bash: 没有那个文件` | 仿真工作区路径不对或没编；`find ~ -name start_simulator.sh` |
| `ros2 launch ... executable not found` | 没重编，或 `rune_gimbal_node` 没找到 `rm_energy_interfaces` |
| 视频源报“打不开视频” | `video_path` 不对；`ls -l ~/demo_01/big_11.avi` |
| 识别日志没有 `处理 ...` | 视频源没发图，或 `compressed_topic` 对不上 |
| 调试图窗口没弹 | `python3 -c 'import cv2'`；没装就 `sudo apt install python3-opencv` |
| 调试图一顿一顿 | `debug_fps` 调大（15~20），别调小 |
| 调试图糊 | `debug_scale` 太小；1.0 = 720×540 |
| 云台不开火 | 看 `abs` 的 `r/omega`、`aim` 的 `miss` 是否进 `fire_miss_m` |
| 改了参数没生效 | 参数只在启动时读；重启节点 |

