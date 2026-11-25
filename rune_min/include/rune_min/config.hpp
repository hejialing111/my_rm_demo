#pragma once

#include <opencv2/core.hpp>

namespace rune {

// 图像预处理（去畸变 / ROI / 缩放）需要的配置。
//
// 能耗机关的几何、弹道、EKF 噪声这些参数不在这里：
//   - 检测阈值在 BladeDetectConfig（blade_detector.hpp）
//   - 测距用 PnP（feature_diameter_m + 相机内参），配置在 rune_exam.yaml
//   - 云台/弹道参数在 gimbal_node 里
struct RuneConfig {
    // ---------------- 相机 ----------------
    // 空 = 没有标定，去畸变自动跳过（零开销）。
    cv::Mat K;
    cv::Mat D;

    // ---------------- 预处理 ----------------
    // K 有值且 D 非全零时才真的去畸变。
    bool enable_undistort = true;

    // <1 = 先把图缩小再检测。角度对等比缩放不敏感，但 HSV/形态学/找轮廓会快很多。
    double detect_scale = 1.0;

    // ROI：只在这块区域里检测，以 r_center_px 为中心（原图像素）。
    // {0,0} = 整幅图。至少要比能量机关直径大一圈，否则灯臂转到边缘会被裁掉。
    cv::Size roi_size{0, 0};

    // R 中心在**原图**中的像素位置；填负数时退回图像中心。
    cv::Point2f r_center_px{-1.0f, -1.0f};
};

}  // namespace rune
