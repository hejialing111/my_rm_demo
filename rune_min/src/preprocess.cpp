#include "rune_min/preprocess.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
//有ROI,去畸变，换分辨率时用
namespace rune {
namespace {
//原始图裁ROI,缩放到720*540，去畸变
bool has_distortion(const cv::Mat& K, const cv::Mat& D) {
    if (K.empty() || D.empty()) return false;
    if (K.rows != 3 || K.cols != 3) return false;
    return cv::norm(D, cv::NORM_INF) > 0.0;
}

double ticks_to_ms(std::int64_t ticks) {
    return static_cast<double>(ticks) * 1000.0 / cv::getTickFrequency();
}

//裁ROI
// cv::Rect make_roi(cv::Size raw_size, cv::Point2f center, cv::Size want) {
//     const cv::Rect full(0, 0, raw_size.width, raw_size.height);
//     if (want.width <= 0 || want.height <= 0) return full;

//     const int w = std::min(want.width, full.width);
//     const int h = std::min(want.height, full.height);
//     int x = static_cast<int>(std::lround(center.x)) - w / 2;
//     int y = static_cast<int>(std::lround(center.y)) - h / 2;
//     x = std::clamp(x, 0, full.width - w);
//     y = std::clamp(y, 0, full.height - h);
//     return cv::Rect(x, y, w, h);
// }

}  // namespace

FramePreprocessor::FramePreprocessor(const RuneConfig& config) : config_(config) {
    // 统一成 CV_64F，后面就能放心 at<double>()；标定文件读进来的 dtype 不一定是 double。
    if (!config_.K.empty()) {
        cv::Mat k64;
        config_.K.convertTo(k64, CV_64F);
        config_.K = k64;
    }
    if (!config_.D.empty()) {
        cv::Mat d64;
        config_.D.convertTo(d64, CV_64F);
        config_.D = d64;
    }

    // 缩放留个安全范围：太小没意义，太大会把假阳性放大。
    scale_ = std::clamp(config_.detect_scale, 0.1, 4.0);
    use_undistort_ = config_.enable_undistort && has_distortion(config_.K, config_.D);
}
//尺寸没变就直接返回
void FramePreprocessor::rebuild_if_needed(cv::Size raw_size) {
    if (raw_size == cached_raw_size_) return;
    cached_raw_size_ = raw_size;

    // 配置里的 R 中心是“原图像素”；填负数就用图心。
    cv::Point2f r_center_raw = config_.r_center_px;
    if (r_center_raw.x < 0.0f || r_center_raw.y < 0.0f) {
        r_center_raw = cv::Point2f(raw_size.width * 0.5f, raw_size.height * 0.5f);
    }

    // roi_ = make_roi(raw_size, r_center_raw, config_.roi_size);
    roi_ = cv::Rect(0, 0, raw_size.width, raw_size.height);

    view_size_ = cv::Size(
        std::max(1, static_cast<int>(std::lround(roi_.width * scale_))),
        std::max(1, static_cast<int>(std::lround(roi_.height * scale_))));

    if (use_undistort_) {
        // 关键：把新相机的 cx/cy 平移到 ROI 左上角，再整体按 scale 缩放。
        // 这样 remap 的“目标图”就是 ROI 这块小图，只插值一次、只写 ROI 那么大。
        const double fx = config_.K.at<double>(0, 0);
        const double fy = config_.K.at<double>(1, 1);
        const double skew = config_.K.at<double>(0, 1);
        const double cx = config_.K.at<double>(0, 2);
        const double cy = config_.K.at<double>(1, 2);

        new_k_ = (cv::Mat_<double>(3, 3) <<
            fx * scale_, skew * scale_, (cx - roi_.x) * scale_,
            0.0, fy * scale_, (cy - roi_.y) * scale_,
            0.0, 0.0, 1.0);

        cv::initUndistortRectifyMap(config_.K, config_.D, cv::noArray(), new_k_,
                                    view_size_, CV_32FC1, map1_, map2_);
    } else {
        map1_.release();
        map2_.release();
        new_k_.release();
    }

    r_center_view_ = map_point(r_center_raw);
}
//空图返回空
const cv::Mat& FramePreprocessor::apply(const cv::Mat& raw) {
    if (raw.empty()) {
        view_.release();
        return view_;
    }
//开始计时
    const std::int64_t t0 = cv::getTickCount();
    //尺寸变了就重建
    rebuild_if_needed(raw.size());

    const bool full_frame =
        roi_.x == 0 && roi_.y == 0 &&
        roi_.width == raw.cols && roi_.height == raw.rows;

    if (use_undistort_) {
        // 映射表已经包含 ROI 平移和缩放，直接整幅 remap 就得到 ROI 视图。
        cv::remap(raw, view_, map1_, map2_, cv::INTER_LINEAR);
    } else if (std::abs(scale_ - 1.0) < 1e-9) {
        view_ = full_frame ? raw : raw(roi_);  // 零拷贝：视图是原图 / 原图的一块
    } else {
        cv::resize(raw(roi_), view_, view_size_, 0, 0, cv::INTER_AREA);//实际用的是resize,缩小图像时区
    }

    cost_ms_ = ticks_to_ms(cv::getTickCount() - t0);
    return view_;
}

cv::Point2f FramePreprocessor::undistort_point(const cv::Point2f& raw_px) const {
    if (!use_undistort_) return raw_px;
    const std::vector<cv::Point2f> src{raw_px};
    std::vector<cv::Point2f> dst;
    // P = new_k_ 已经含 ROI 平移，所以结果天然就是“ROI 视图坐标”。
    cv::undistortPoints(src, dst, config_.K, config_.D, cv::noArray(), new_k_);
    return dst.empty() ? raw_px : dst.front();
}

cv::Point2f FramePreprocessor::map_point(const cv::Point2f& raw_px) const {
    if (use_undistort_) return undistort_point(raw_px);
    return cv::Point2f(
        static_cast<float>((raw_px.x - roi_.x) * scale_),
        static_cast<float>((raw_px.y - roi_.y) * scale_));
}
}