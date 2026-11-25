#pragma once

#include <cstdint>
#include <vector>

#include "rune_min/blade_tracker.hpp"

namespace rune {

struct TrackSelectConfig {
    // hole_count >= 2 “待击打”。只影响“换靶时优先挑谁”，
    int pending_min_holes = 2;

    // 当前目标连续丢失这么多帧才允许换靶（比 BladeTracker::max_missed 小）。
    int switch_after_missed = 3;

    // 当前目标连续这么多帧被判成“已激活”才换靶。
    // hole_count 在阈值附近会抖，没有这个确认就会每帧换。
    int activate_confirm = 3;
};

struct TrackSelectResult {
    int index = -1;             // 在 tracks 里的下标；-1 = 本帧没有可打目标
    bool continued = false;     // 还是上一帧那一块
    bool switched = false;      // 本帧换了目标
    bool all_activated = false; // 全部被判成“已激活”，走了兜底
    bool target_activated = false;  // 是因为“打过了”而换的（不是丢失）
    bool pending = false;       // 选中的这块是否“待击打”（hole_count 达标）
    std::uint16_t track_id = 0; // 选中/保持的 track id
};

// 选靶：从跟踪器维护的 track 里挑“这一帧该打哪一块”。
//
// 只认 track id，不认 hole_count：粘滞由身份保证，状态量抖动不会导致换靶。
class TrackSelector {
public:
    explicit TrackSelector(const TrackSelectConfig& config = {});

    TrackSelectResult select(const std::vector<BladeTrack>& tracks);

    std::uint16_t aimed_id() const { return aimed_id_; }
    void reset();

private:
    static bool is_pending(const BladeTrack& track, int min_holes);

    TrackSelectConfig cfg_;
    std::uint16_t aimed_id_ = 0;
    int low_pending_frames_ = 0;  // 当前目标连续“非待击打”的帧数
};

}  // namespace rune
