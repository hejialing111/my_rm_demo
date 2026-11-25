#pragma once

#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include "rune_min/blade_detector.hpp"

namespace rune {

// 单个目标的跟踪状态（track-by-detection）。
//
// 设计要点：**身份(id) 和 状态(hole_count) 是分开的**。
// hole_count 只是 track 上的一个属性，用来判断“待击打/已激活”，
// 绝不参与“这一帧是谁”的判定 —— 否则状态一抖，身份就丢，云台就会在两块靶
// 之间来回甩（2026-11 仿真里实测 id 每帧自增，就是这个原因）。
struct BladeTrack {
    std::uint16_t id = 0;      // 稳定跟踪号，从 1 开始；0 = 无效
    bool confirmed = false;    // 连续命中 confirm_hits 帧后才对外可见
    int hits = 0;              // 累计关联次数
    int missed = 0;            // 连续未关联帧数
    double stamp = 0.0;        // 最后一次更新（或状态转移）的时间戳，秒

    cv::Point2f center{0.0f, 0.0f};    // 图像坐标（像素）
    cv::Point2f velocity{0.0f, 0.0f};  // 图像系匀速模型，px/s
    float radius_px = 0.0f;            // 椭圆半长轴
    float semi_minor_px = 0.0f;
    float angle_rad = 0.0f;            // 长轴方向
    double area_px = 0.0;
    int hole_count = 0;
    float confidence = 0.0f;
};

struct BladeTrackerConfig {
    // 连续这么多帧没关联上就删除 track。
    // 比 TrackSelectConfig::switch_after_missed 大，保证“先保留、后换靶”。
    int max_missed = 5;

    // 连续关联这么多帧才算 confirmed，避免单帧噪点生成一个假目标。
    int confirm_hits = 2;

    int max_tracks = 8;

    // 关联门限：max(min_gate_px, gate_radius_ratio × 半径,
    //               gate_speed_factor × |v| × dt)
    double gate_radius_ratio = 2.5;
    double gate_speed_factor = 1.5;
    double min_gate_px = 12.0;

    // 时间戳异常（dt > 该值）时按该值封顶，避免一步把速度算飞。
    double max_dt_s = 0.30;

    // 速度 EMA 系数：越大越跟手，越小越平滑。
    double velocity_alpha = 0.5;

    // 单帧位移换算出的速度超过它，就认为是野值，不更新速度。
    double max_speed_px_s = 4000.0;
};

// 亮靶跟踪器：每帧“预测 -> 关联 -> 更新 / coast”，身份跨帧保持。
//
// 和 TrackSelector 的分工：
//   BladeTracker  —— 这一帧的检测对应上一帧的哪一块（身份）；
//   TrackSelector —— 这么多 track 里该打哪一个（策略）。
// 两者都不依赖 ROS，可离线单测。
class BladeTracker {
public:
    explicit BladeTracker(const BladeTrackerConfig& config = {});

    // 喂一帧检测结果。detections 可以为空（整帧没检出），
    // 此时所有 track 只做状态转移、missed++。
    void update(const std::vector<Blade>& detections, double stamp);

    const std::vector<BladeTrack>& tracks() const { return tracks_; }

    // 清空所有状态（换相机/重开机关时用）。
    void reset();

private:
    BladeTrackerConfig cfg_;
    std::vector<BladeTrack> tracks_;
    std::uint16_t next_id_ = 1;
};

}  // namespace rune
