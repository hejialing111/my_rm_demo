#pragma once

#include <vector>

#include <opencv2/core.hpp>

namespace rune {

struct PlanarPnPResult {
    bool valid = false;
    cv::Vec3d rvec{0, 0, 0};//旋转向量
    cv::Vec3d tvec{0, 0, 0};//平移向量
    double reproj_error = 0.0;
};

// 通用平面 PnP：物体点 -> 图像点。
// 用于 R 标/装甲板四角等已知平面矩形。
PlanarPnPResult solve_planar_pnp(
    const std::vector<cv::Point3f>& object_points,
    const std::vector<cv::Point2f>& image_points,
    const cv::Mat& K,
    const cv::Mat& D);

// 已知直径的圆盘（靶面）的位姿解算。
//
// 输入是检测出来的椭圆：中心、半长轴、半短轴、长轴方向（弧度）。
// 椭圆的长轴/短轴四个端点，正好对应圆盘圆周上互相垂直的四个点，
// 所以直接拿来做平面 PnP，输出 tvec 就是**靶心在相机系下的位置**。
//
// 相比“焦距×直径/像素半径”的比例法，PnP 用上了整个轮廓的形状，
// 同时还能给出盘面朝向（rvec）；代价是要相机内参和一次迭代求解。
PlanarPnPResult solve_disc_pnp(const cv::Point2f& center_px,
                               double semi_major_px,
                               double semi_minor_px,
                               double angle_rad,
                               double diameter_m,
                               const cv::Mat& K,
                               const cv::Mat& D);

}  // namespace rune
