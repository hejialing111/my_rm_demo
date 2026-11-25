#pragma once

#include <cstddef>
#include <deque>
#include <string>

namespace rune {

// 一个带时间戳的云台姿态样本。
//
// 角度约定和 GimbalCommand 一致：**相对初始姿态的绝对偏移**
//   yaw  > 0 向右；pitch > 0 向上；单位 rad。
// 相机当前的光轴仰角 = initial_elevation_rad + pitch。
struct PoseSample {
    double t = 0.0;
    double yaw = 0.0;
    double pitch = 0.0;
};

// 姿态时间序列：按时间排序保存最近若干秒，支持线性插值查询。
//
// 两种输入源都用它：
//   1) 命令历史（cmd_pose_）：每次发 GimbalCommand 时记一条，
//      收到图像时按 header.stamp 反查"采集那一刻指令到哪";
//   2) 真实姿态（oracle_pose_）：订阅仿真器的 /rm_energy/debug/labels
//      或实车 IMU，直接记"那一刻云台实际在哪"。
//
// 为什么要插值：50 Hz 的命令历史是 20 ms 一格，直接取"≤t 的最后一条"相当于
// 给姿态额外加了一个 [0,20) ms 的随机滞后。对 1 rad/s 的跟踪，20 ms 就是
// 0.02 rad ≈ 10 cm @5 m —— 和命中半径同量级。线性插值把这一项误差压到接近 0。
class PoseHistory {
public:
    explicit PoseHistory(std::size_t max_samples = 512, double max_age_s = 6.0);

    // 追加一个样本。时间必须严格递增；重复/回退的样本会被丢弃（同一时刻的会覆盖），
    // 因为仿真里偶尔会重发同一帧。
    void add(double t, double yaw, double pitch);

    void clear();
    bool empty() const { return buf_.empty(); }
    std::size_t size() const { return buf_.size(); }

    // 查 t 时刻的姿态（线性插值）。
    //
    // 返回 true = t 落在历史覆盖区间 [最早, 最新] 内，结果可信。
    // 返回 false = 历史为空 / t 比最早的样本还早 / t 比最新的样本还新。
    //              这三种情况下仍会把端点值写进 yaw/pitch（历史非空时），
    //              调用方据此决定是"凑合用"还是"换别的源"。
    bool at(double t, double& yaw, double& pitch) const;

    // 最新一条样本。用于"现在这一刻云台在哪"——真实姿态本来就是零阶保持的，
    // 最新一条就是当前实际姿态。
    bool latest(PoseSample& out) const;

private:
    std::deque<PoseSample> buf_;
    std::size_t max_samples_;
    double max_age_s_;
};

// 解析 /rm_energy/debug/labels 的 JSON，取 stamp_ns / yaw_rad / pitch_rad。
//
// 只认这三个键，其余字段一律忽略；缺键、数值非法或时间戳非正都返回 false。
// 这是一个"够用就好"的关键字扫描器，不引入 JSON 依赖，也不做通用性承诺：
// 仿真器的 labels 是扁平对象（Unity JsonUtility 序列化的），没有嵌套转义。
bool parse_pose_labels(const std::string& json, PoseSample& out);

}  // namespace rune
