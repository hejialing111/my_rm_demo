#!/usr/bin/env bash
set -eo pipefail

release_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
player="$release_dir/Simulator/RMEnergyHit.x86_64"
workspace="$release_dir/ros2_ws"

if [[ ! -x "$player" ]]; then
  echo "未找到仿真程序：$player" >&2
  exit 1
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "请先安装 ROS 2 Humble。" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
if [[ ! -f "$workspace/install/setup.bash" ]]; then
  (cd "$workspace" && colcon build --cmake-args \
    -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3)
fi
source "$workspace/install/setup.bash"
existing_camera_publishers="$(ros2 topic info /rm_energy/camera/image_raw/compressed 2>/dev/null | awk '/Publisher count:/ { print $3 }' || true)"
if [[ "${existing_camera_publishers:-0}" -gt 0 ]]; then
  echo "检测到 ${existing_camera_publishers} 个已有相机中继。请先关闭旧的仿真/ROS 桥接，避免 UDP 争抢造成图像卡顿。" >&2
  exit 1
fi
setsid ros2 launch rm_energy_bridge simulator.launch.py &
bridge_pid=$!
cleanup() { kill -INT -- "-$bridge_pid" 2>/dev/null || true; wait "$bridge_pid" 2>/dev/null || true; }
trap cleanup EXIT
"$player" "$@"
