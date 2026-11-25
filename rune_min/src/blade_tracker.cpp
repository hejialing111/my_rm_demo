#include "rune_min/blade_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace rune {
namespace {
//把观测到的数据转为目标轨迹


// 时间戳可能重复、回退或跳变（Unity 的 stamp_ns、多话题合流都会这样）。
// 统一兜底：非正或过大都夹到 [0, max_dt]，避免一步把速度算飞。
double frame_dt(double stamp, double last_stamp, double max_dt) {
    const double dt = stamp - last_stamp;
    if (!(dt > 0.0)) return 0.0;
    if (!(dt <= max_dt)) return max_dt;
    return dt;
}

}  // namespace
//构造函数，目标靶子的vector容器
BladeTracker::BladeTracker(const BladeTrackerConfig& config) : cfg_(config) {
    if (cfg_.max_missed < 0) cfg_.max_missed = 0;
    if (cfg_.confirm_hits < 1) cfg_.confirm_hits = 1;
    if (cfg_.max_tracks < 1) cfg_.max_tracks = 1;
    if (cfg_.min_gate_px < 1.0) cfg_.min_gate_px = 1.0;
    if (cfg_.max_dt_s <= 0.0) cfg_.max_dt_s = 0.30;
    if (cfg_.velocity_alpha <= 0.0 || cfg_.velocity_alpha > 1.0) cfg_.velocity_alpha = 0.5;
    if (cfg_.max_speed_px_s <= 0.0) cfg_.max_speed_px_s = 4000.0;
}
//清空所有track，id从1开始
void BladeTracker::reset() {
    tracks_.clear();
    next_id_ = 1;
}
//在update上传入检测到的亮靶和时间戳
void BladeTracker::update(const std::vector<Blade>& detections, double stamp) {
    const int n_tracks = static_cast<int>(tracks_.size());
    const int n_dets = static_cast<int>(detections.size());

    //用上一帧的位置预测这一帧位置
    std::vector<double> dts(static_cast<std::size_t>(n_tracks), 0.0);
    std::vector<cv::Point2f> preds(static_cast<std::size_t>(n_tracks));
    for (int ti = 0; ti < n_tracks; ++ti) {
        const std::size_t i = static_cast<std::size_t>(ti);
        dts[i] = frame_dt(stamp, tracks_[i].stamp, cfg_.max_dt_s);
        const float dt = static_cast<float>(dts[i]);
        preds[i] = cv::Point2f(tracks_[i].center.x + tracks_[i].velocity.x * dt,//在图像坐标系下移动的角速度，像素为单位
                               tracks_[i].center.y + tracks_[i].velocity.y * dt);
    }

    //这一帧检测到的靶子是上一阵的哪一块
    struct Candidate {
        double dist;//预测到检测中心的距离
        int track;//跟踪
        int det;//检测
    };
    std::vector<Candidate> candidates;
    for (int ti = 0; ti < n_tracks; ++ti) {
        for (int di = 0; di < n_dets; ++di) {
            const Blade& det = detections[static_cast<std::size_t>(di)];
            if (!det.valid) continue;

            const std::size_t i = static_cast<std::size_t>(ti);
            const double dist = cv::norm(preds[i] - det.center);

            //设一个门线，防止检测到的东西太离谱，在合理范围内比对
            double gate = cfg_.min_gate_px;
            gate = std::max(gate, cfg_.gate_radius_ratio *
                                      std::max<double>(tracks_[i].radius_px, det.radius_px));
            gate = std::max(gate, cfg_.gate_speed_factor * cv::norm(tracks_[i].velocity) * dts[i]);
            if (dist <= gate) {
                candidates.push_back(Candidate{dist, ti, di});
            }
        }
    }

    //这里贪心把每一个匹配到的距离进行排序
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& a, const Candidate& b) {
                  if (a.dist != b.dist) return a.dist < b.dist;
                  if (a.track != b.track) return a.track < b.track;
                  return a.det < b.det;
              });
//track匹配上检测，检测匹配上track
    std::vector<int> track_to_det(static_cast<std::size_t>(n_tracks), -1);
    std::vector<int> det_to_track(static_cast<std::size_t>(n_dets), -1);
    //双向确定则成功
    for (const Candidate& c : candidates) {
        if (track_to_det[static_cast<std::size_t>(c.track)] >= 0) continue;
        if (det_to_track[static_cast<std::size_t>(c.det)] >= 0) continue;
        track_to_det[static_cast<std::size_t>(c.track)] = c.det;
        det_to_track[static_cast<std::size_t>(c.det)] = c.track;
    }
//匹配上则更新现在的真实位置
    for (int ti = 0; ti < n_tracks; ++ti) {
        const int di = track_to_det[static_cast<std::size_t>(ti)];
        if (di < 0) continue;

        BladeTrack& track = tracks_[static_cast<std::size_t>(ti)];
        const Blade& det = detections[static_cast<std::size_t>(di)];
        const double dt = dts[static_cast<std::size_t>(ti)];
//匹配上用跟踪的和现在的位置算出速度
        if (dt > 1e-4) {
            const cv::Point2f v(static_cast<float>((det.center.x - track.center.x) / dt),
                                static_cast<float>((det.center.y - track.center.y) / dt));
            if (std::hypot(v.x, v.y) <= cfg_.max_speed_px_s) {

                //速度EMA对历史观测做平均
                const float a = static_cast<float>(cfg_.velocity_alpha);
                track.velocity = cv::Point2f(a * v.x + (1.0f - a) * track.velocity.x,
                                             a * v.y + (1.0f - a) * track.velocity.y);
            }
        }
//更新
        track.center = det.center;
        track.radius_px = det.radius_px;
        track.semi_minor_px = det.semi_minor_px;
        track.angle_rad = det.angle_rad;
        track.area_px = det.area_px;
        track.hole_count = det.hole_count;
        track.confidence = det.confidence;
        track.missed = 0;
        ++track.hits;
        track.stamp = stamp;
        if (!track.confirmed && track.hits >= cfg_.confirm_hits) {
            track.confirmed = true;
        }
    }
//若是，没匹配上则 再往外推几步
    for (int ti = 0; ti < n_tracks; ++ti) {
        if (track_to_det[static_cast<std::size_t>(ti)] >= 0) continue;
        BladeTrack& track = tracks_[static_cast<std::size_t>(ti)];
        const float dt = static_cast<float>(dts[static_cast<std::size_t>(ti)]);
        track.center.x += track.velocity.x * dt;
        track.center.y += track.velocity.y * dt;
        ++track.missed;
        track.stamp = stamp;
    }

    //检测到新的则新建一个跟踪
    for (int di = 0; di < n_dets; ++di) {
        if (det_to_track[static_cast<std::size_t>(di)] >= 0) continue;
        const Blade& det = detections[static_cast<std::size_t>(di)];
        if (!det.valid) continue;

        BladeTrack track;
        track.id = next_id_++;
        if (next_id_ == 0) next_id_ = 1;  // 0 留给“无效”，回绕时跳过
        track.confirmed = (cfg_.confirm_hits <= 1);
        track.hits = 1;
        track.missed = 0;
        track.stamp = stamp;
        track.center = det.center;
        track.radius_px = det.radius_px;
        track.semi_minor_px = det.semi_minor_px;
        track.angle_rad = det.angle_rad;
        track.area_px = det.area_px;
        track.hole_count = det.hole_count;
        track.confidence = det.confidence;
        tracks_.push_back(track);
    }

    //超时则放弃
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                 [this](const BladeTrack& track) {
                                     return track.missed > cfg_.max_missed;
                                 }),
                  tracks_.end());

    // 最多跟踪max_tracks个
    while (static_cast<int>(tracks_.size()) > cfg_.max_tracks) {
        std::size_t worst = 0;
        for (std::size_t i = 1; i < tracks_.size(); ++i) {
            const BladeTrack& a = tracks_[i];
            const BladeTrack& b = tracks_[worst];
            bool a_worse = false;
            if (a.confirmed != b.confirmed) {
                a_worse = !a.confirmed;
            } else if (a.missed != b.missed) {
                a_worse = a.missed > b.missed;
            }
            if (a_worse) worst = i;
        }
        tracks_.erase(tracks_.begin() + static_cast<std::ptrdiff_t>(worst));
    }
}

}  // namespace rune
