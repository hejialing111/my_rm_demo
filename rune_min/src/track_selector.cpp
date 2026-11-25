#include "rune_min/track_selector.hpp"

#include <cstddef>

namespace rune {

TrackSelector::TrackSelector(const TrackSelectConfig& config) : cfg_(config) {
    if (cfg_.pending_min_holes < 0) cfg_.pending_min_holes = 0;
    if (cfg_.switch_after_missed < 1) cfg_.switch_after_missed = 1;
    if (cfg_.activate_confirm < 1) cfg_.activate_confirm = 1;
}

void TrackSelector::reset() {
    aimed_id_ = 0;
    low_pending_frames_ = 0;
}
//若返回1则说明内部洞多，也就是待击打
bool TrackSelector::is_pending(const BladeTrack& track, int min_holes) {
    return track.hole_count >= min_holes;
}

TrackSelectResult TrackSelector::select(const std::vector<BladeTrack>& tracks) {
    TrackSelectResult result;

    // 当前在打的那一块
    int aimed = -1;
    if (aimed_id_ != 0) {
        for (std::size_t i = 0; i < tracks.size(); ++i) {
            if (tracks[i].id == aimed_id_ && tracks[i].confirmed) {
                aimed = static_cast<int>(i);
                break;
            }
        }
    }

    if (aimed >= 0) {
        const BladeTrack& track = tracks[static_cast<std::size_t>(aimed)];
        const bool pending = is_pending(track, cfg_.pending_min_holes);//内部hole大于2则返回1也就是待击打
        // 连续 missed 达到 switch_after_missed 才算“丢了”，允许换靶。
        const bool alive = track.missed < cfg_.switch_after_missed;

        //正常情况：还在、还是待击打就打这个，不会因为面积的变化就换靶子，按照id标定待击打目标
        if (alive && pending) {
            low_pending_frames_ = 0;
            result.index = aimed;
            result.continued = true;
            result.pending = true;
            result.track_id = track.id;
            return result;
        }

        //寻找有没有可以击打的靶子要求confirm missed==0  待击打
        bool has_pending_other = false;
        if (alive && !pending) {
            for (std::size_t i = 0; i < tracks.size(); ++i) {
                if (static_cast<int>(i) == aimed) continue;//当前就是打击后的目标靶子
                if (!tracks[i].confirmed || tracks[i].missed != 0) continue;//并非是i准确的confirm后的靶子或者是不是新测量的位置，也就是不用外推出来的位置
                if (is_pending(tracks[i], cfg_.pending_min_holes)) {
                    has_pending_other = true;
                    break;
                }
            }
        }

        if (alive && !pending && !has_pending_other) {//打完发现没有换的目标了
            //这时换靶只会在两块之间来回跳，所以在没有可击打的目标出现前保持当前目标。
            low_pending_frames_ = 0;
            result.index = aimed;
            result.continued = true;
            result.track_id = track.id;
            result.all_activated = true;
            return result;
        }

        if (alive && !pending) {//打完后未判断是否是真的稳定确定已激活就换，若非则判定为不正确的抖动，若是则换另一个可打的
            ++low_pending_frames_;
            if (low_pending_frames_ < cfg_.activate_confirm) {//数几帧防止云台抖动
                result.index = aimed;
                result.continued = true;
                result.track_id = track.id;
                return result;
            }
            result.target_activated = true;  // 连续 N 帧都非待击打，认定打过了
        }// alive == false（丢太久）也走到这里换靶。
    }



    //换靶子，找面积最大的或者全局面积最大的
    int best_pending = -1;
    int best_all = -1;
    for (std::size_t i = 0; i < tracks.size(); ++i) {
        const BladeTrack& track = tracks[i];
        if (!track.confirmed || track.missed != 0) continue;

        const bool pending = is_pending(track, cfg_.pending_min_holes);
        const auto better_than = [&tracks](const BladeTrack& cand, int best) {
            if (best < 0) return true;
            const BladeTrack& current = tracks[static_cast<std::size_t>(best)];
            if (cand.area_px != current.area_px) return cand.area_px > current.area_px;
            return cand.hits > current.hits;
        };
        if (pending && better_than(track, best_pending)) best_pending = static_cast<int>(i);
        if (better_than(track, best_all)) best_all = static_cast<int>(i);
    }
    const int picked = (best_pending >= 0) ? best_pending : best_all;
    result.all_activated = (best_pending < 0);
    if (picked < 0) {
        // 这一帧没有任何可用目标：保持 aimed_id_，等它回来或等新目标。
        result.track_id = aimed_id_;
        return result;
    }


    const BladeTrack& chosen = tracks[static_cast<std::size_t>(picked)];
    result.index = picked;
    result.track_id = chosen.id;
    result.pending = is_pending(chosen, cfg_.pending_min_holes);
    result.switched = (chosen.id != aimed_id_);
    result.continued = (!result.switched && aimed_id_ != 0);
    aimed_id_ = chosen.id;
    low_pending_frames_ = 0;
    return result;
}

}  // namespace rune
