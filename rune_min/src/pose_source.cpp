#include "rune_min/pose_source.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdlib>

namespace rune {
namespace {

// 时间戳比较的容差。epoch 秒约 1.7e9，double 的 ulp 在 2~5e-7 s 量级，
// 所以 1e-6 s 足够吸收"同一个 stamp 用两种路径算出来"的舍入差。
constexpr double kTimeEps = 1e-6;

// 在扁平 JSON 里找 "key": <number>，把数值读进 out。
// 找不到 key / 冒号 / 合法数字都返回 false。
bool find_json_number(const std::string& json, const char* key, double& out) {
    const std::string needle = std::string("\"") + key + "\"";
    std::size_t pos = json.find(needle);
    if (pos == std::string::npos) return false;

    pos = json.find(':', pos + needle.size());
    if (pos == std::string::npos) return false;
    ++pos;

    while (pos < json.size() &&
           std::isspace(static_cast<unsigned char>(json[pos]))) {
        ++pos;
    }
    if (pos >= json.size()) return false;

    errno = 0;
    char* end = nullptr;
    const double value = std::strtod(json.c_str() + pos, &end);
    if (end == json.c_str() + pos || errno == ERANGE || !std::isfinite(value)) {
        return false;
    }
    out = value;
    return true;
}

}  // namespace

PoseHistory::PoseHistory(std::size_t max_samples, double max_age_s)
    : max_samples_(max_samples < 2 ? 2 : max_samples),
      max_age_s_(max_age_s > 0.0 ? max_age_s : 6.0) {}

void PoseHistory::add(double t, double yaw, double pitch) {
    if (!std::isfinite(t) || !std::isfinite(yaw) || !std::isfinite(pitch)) return;

    if (!buf_.empty()) {
        // 同一时刻重发：覆盖。时间回退：丢弃（仿真里偶发，不能让它污染插值）。
        if (t <= buf_.back().t) {
            if (std::fabs(t - buf_.back().t) <= kTimeEps) {
                buf_.back().yaw = yaw;
                buf_.back().pitch = pitch;
            }
            return;
        }
    }

    buf_.push_back(PoseSample{t, yaw, pitch});

    while (buf_.size() > max_samples_) buf_.pop_front();
    while (buf_.size() > 1 && (buf_.back().t - buf_.front().t) > max_age_s_) {
        buf_.pop_front();
    }
}

void PoseHistory::clear() { buf_.clear(); }

bool PoseHistory::at(double t, double& yaw, double& pitch) const {
    if (buf_.empty()) return false;

    if (t < buf_.front().t - kTimeEps) {
        yaw = buf_.front().yaw;
        pitch = buf_.front().pitch;
        return false;
    }
    if (t > buf_.back().t + kTimeEps) {
        yaw = buf_.back().yaw;
        pitch = buf_.back().pitch;
        return false;
    }

    // 第一个时间 >= t 的样本
    const auto it = std::lower_bound(
        buf_.begin(), buf_.end(), t,
        [](const PoseSample& s, double v) { return s.t < v; });

    if (it == buf_.begin()) {  // t 就是最早那一帧
        yaw = it->yaw;
        pitch = it->pitch;
        return true;
    }
    if (it == buf_.end()) {  // t 就是最新那一帧（或差在 eps 内）
        yaw = buf_.back().yaw;
        pitch = buf_.back().pitch;
        return true;
    }

    const PoseSample& b = *it;
    const PoseSample& a = *(it - 1);
    const double span = b.t - a.t;
    if (!(span > kTimeEps)) {
        yaw = b.yaw;
        pitch = b.pitch;
        return true;
    }
    const double w = (t - a.t) / span;
    yaw = a.yaw + w * (b.yaw - a.yaw);
    pitch = a.pitch + w * (b.pitch - a.pitch);
    return true;
}

bool PoseHistory::latest(PoseSample& out) const {
    if (buf_.empty()) return false;
    out = buf_.back();
    return true;
}

bool parse_pose_labels(const std::string& json, PoseSample& out) {
    if (json.empty()) return false;

    double stamp_ns = 0.0;
    double yaw = 0.0;
    double pitch = 0.0;
    if (!find_json_number(json, "stamp_ns", stamp_ns)) return false;
    if (!find_json_number(json, "yaw_rad", yaw)) return false;
    if (!find_json_number(json, "pitch_rad", pitch)) return false;
    if (!(stamp_ns > 0.0)) return false;

    out.t = stamp_ns * 1e-9;
    out.yaw = yaw;
    out.pitch = pitch;
    return true;
}

}  // namespace rune
