#pragma once

#include <string>

#include <opencv2/core.hpp>

#include "rune_min/config.hpp"

namespace rune {

// 图像预处理：把“相机出来的原图”变成“检测用的图”。
//
//   raw ──[ROI 裁剪]──[可选 去畸变]──[可选 缩放]──▶ view
//
// 裁出关心的区域、修镜头畸变、按比例缩小、把点换算到 view 坐标。
//
// 为什么需要它：
//   - K/D 之前配了没人用，画面边缘的角度误差会直接进 EKF；
//   - 1280x720 全分辨率跑 HSV + 形态学太贵。缩一半（角度几乎不变）或只算 ROI，
//     都能把耗时砍一个量级；
//   - 裁剪/缩放/去畸变之后像素坐标全变了，r_center_px、面积阈值、核大小都得跟着换算，
//     这些换算集中在这里，别散到 detector 里；
//   - 只有一处知道“原图和检测图怎么对应”，换输入源/换分辨率时不容易错。
//
// 实现上的两个关键点：
//   1. 去畸变映射表只在尺寸/ROI 变化时重建一次，不是每帧现算；
//   2. 去畸变 + ROI + 缩放合成“一次 remap”：把新相机的 cx/cy 平移到 ROI 原点，
//      这样直接对原图 remap 出目标小图，只插值一次、只写 ROI 那么大。
class FramePreprocessor {
public:
    explicit FramePreprocessor(const RuneConfig& config);

    // 预处理一帧，返回检测视图。
    //
    // 注意：完全不需要处理时为了零拷贝，返回的是 raw 的浅拷贝（或它的 ROI 视图），
    // 生命周期跟随传进来的 raw。要长期持有请自己 clone。
    const cv::Mat& apply(const cv::Mat& raw);

    // 原图像素 -> 视图像素。r_center_px 就是靠它换算到检测坐标系的。
    cv::Point2f map_point(const cv::Point2f& raw_px) const;

    // R 中心在视图里的位置（配置填负数时自动取图心），每次尺寸变化时算一次。
    const cv::Point2f& r_center_view() const { return r_center_view_; }

    // 实际生效的 ROI（原图坐标）。没配 ROI 时就是整幅图。
    const cv::Rect& roi() const { return roi_; }

    double scale() const { return scale_; }
    bool undistorted() const { return use_undistort_; }

    // 最近一次 apply() 的耗时（毫秒）。
    double last_cost_ms() const { return cost_ms_; }

    const cv::Mat& view() const { return view_; }

private:
    void rebuild_if_needed(cv::Size raw_size);
    cv::Point2f undistort_point(const cv::Point2f& raw_px) const;

    RuneConfig config_;
    bool use_undistort_ = false;
    double scale_ = 1.0;

    cv::Mat view_;         // 检测视图
    cv::Mat map1_, map2_;  // 去畸变映射表（缓存）
    cv::Mat new_k_;        // 缩放 + ROI 平移后的内参，undistortPoints 用

    cv::Size cached_raw_size_{-1, -1};
    cv::Size view_size_{-1, -1};
    cv::Rect roi_{0, 0, 0, 0};
    cv::Point2f r_center_view_{0.0f, 0.0f};
    double cost_ms_ = 0.0;
};

// 读相机内参和畸变系数。两种格式都认：
//   - OpenCV FileStorage（cv::calibrateCamera + cv::FileStorage 存出来的 .yml/.xml）
//   - ROS 的 camera_info.yaml（image_width / camera_matrix / data: [...]，没有 %YAML 头）
// 读不到 / 解析失败一律返回 false，**不会抛异常**，调用方保持原配置即可。
// bool load_camera_calib(const std::string& path, cv::Mat& K, cv::Mat& D);

}  // namespace rune
