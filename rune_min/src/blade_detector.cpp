#include "rune_min/blade_detector.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace rune {
namespace {

//中间小R标，几条小灯壁，反光，靶子自己的同心细环

//这里先用一个小核3
constexpr int kAltKernelPx = 3;
//odd_kenerl函数用来尝试核大小
int odd_kernel(int k) {
    if (k < 1) k = 1;
    if (k % 2 == 0) ++k;
    return k;
}


//make_blade把所有collect函数需要的参数信息都装配在一起
Blade make_blade(const cv::Point2f& center, float radius, double area, float confidence) {
    Blade b;
    b.valid = true;
    b.center = center;
    b.radius_px = radius;
    b.semi_minor_px = radius;
    b.area_px = area;
    b.confidence = confidence;

    b.keypoints[0] = center;
    b.keypoints[1] = cv::Point2f(center.x, center.y - radius);  // 上
    b.keypoints[2] = cv::Point2f(center.x + radius, center.y);  // 右
    b.keypoints[3] = cv::Point2f(center.x, center.y + radius);  // 下
    b.keypoints[4] = cv::Point2f(center.x - radius, center.y);  // 左
    return b;
}


//enclose函数计算暗连通区域的个数判断是否打过了，有ROI，全图坐标经过运算到ROI尺寸
int enclosed_holes(const cv::Mat& mask, const cv::RotatedRect& ellipse) {
    const double half = 0.5 * std::max(ellipse.size.width, ellipse.size.height);
    //这里计算一个ROI区域是半径的2.6倍加四
    const int side = cvRound(half * 2.6) + 4;
    
    const cv::Rect roi =
    //这里计算ROI的范围
        cv::Rect(cvRound(ellipse.center.x) - side / 2, cvRound(ellipse.center.y) - side / 2,
                 side, side) &

        cv::Rect(0, 0, mask.cols, mask.rows);
    if (roi.width < 8 || roi.height < 8) return 0;

    const cv::Mat patch = mask(roi);
    cv::Mat plate = cv::Mat::zeros(patch.size(), CV_8U);
//这里绘制一个椭圆在靶面上
    cv::ellipse(plate,
                cv::Point(cvRound(ellipse.center.x) - roi.x,
                          cvRound(ellipse.center.y) - roi.y),
                cv::Size(std::max(1, cvRound(0.5 * ellipse.size.width)),
                         std::max(1, cvRound(0.5 * ellipse.size.height))),
                ellipse.angle, 0.0, 360.0, 255, cv::FILLED);

    cv::Mat holes;
    //这里的bitwise用于得到靶子面内部的暗区
    cv::bitwise_not(patch, holes);
    cv::bitwise_and(holes, plate, holes);

    cv::Mat labels, stats, centroids;

    //connectedComponentsWithStarts计算连通区域的数量
    const int n = cv::connectedComponentsWithStats(holes, labels, stats, centroids);

//这里计算一个安区的最小面积并统计个数count
    const int min_hole_area =
        std::max(2, static_cast<int>(std::lround(0.0028 * half * half)));

    int count = 0;
    for (int i = 1; i < n; ++i) {
        if (stats.at<int>(i, cv::CC_STAT_AREA) >= min_hole_area) ++count;
    }
    return count;
}

}


//BladeDetector检测器对象
BladeDetector::BladeDetector(const BladeDetectConfig& config) : cfg_(config) {}

//collect函数用一个指定大小的核kenerl把这张掩码里的所有像靶子的东西找出来，追加到out
void BladeDetector::collect(const cv::Mat& raw_mask, int kernel_px, double img_area,
                            bool allow_fallback, std::vector<Blade>& out) const {
                                //这里在odd_kenerl函数中传入kenel_px参数，返回一个奇数的核大小k
    const int k = odd_kernel(kernel_px);

    // 闭运算把靶面的同心细环填成实心圆；开运算去掉零散噪点。
    cv::Mat closed = raw_mask;
    if (k >= 3) {
        const cv::Mat kernel =
            cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
        cv::morphologyEx(raw_mask, closed, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(closed, closed, cv::MORPH_OPEN, kernel);
    }
//findContours查找轮廓，传入比运算后的图像，返回轮廓contours和层级hierarchy,检索只分两层，同一直线只存首尾两点
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(closed, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);


//最小最大面积
    const double min_area = img_area * cfg_.min_area_ratio;
    const double max_area = img_area * cfg_.max_area_ratio;

    Blade fallback;
    double fallback_score = -1.0;

//椭圆筛选

    for (size_t i = 0; i < contours.size(); ++i) {
        // 只处理最外层轮廓（RETR_CCOMP 里父轮廓下标为 -1）
        if (i < hierarchy.size() && hierarchy[i][3] >= 0) continue;

        const auto& contour = contours[i];
        const double area = cv::contourArea(contour);
        if (area < min_area || area > max_area) continue;
        if (contour.size() < 5) continue;  // fitEllipse 需要至少 5 点

        //fitEllipse拟合椭圆，返回一个RotatedRect类型的ellipse，包含中心点、长轴、短轴和旋转角度
        const cv::RotatedRect ellipse = cv::fitEllipse(contour);
        const double semi_major = 0.5 * std::max(ellipse.size.width, ellipse.size.height);
        const double semi_minor = 0.5 * std::min(ellipse.size.width, ellipse.size.height);
        if (semi_major < cfg_.min_radius_px) continue;

        //太扁
        const double aspect = semi_minor / std::max(semi_major, 1e-6);
        if (aspect < cfg_.min_aspect) continue;

        //椭圆形状拟合
        const double ellipse_fill = area / std::max(CV_PI * semi_major * semi_minor, 1e-6);

        //凸度面积
        std::vector<cv::Point> hull;
        cv::convexHull(contour, hull);
        const double hull_area = cv::contourArea(hull);
        //soildity接近1则可信
        const double solidity = hull_area > 1e-6 ? area / hull_area : 0.0;

        const auto confidence =
            static_cast<float>(std::clamp(0.5 * ellipse_fill + 0.5 * solidity, 0.0, 1.0));

        Blade blade =
            make_blade(ellipse.center, static_cast<float>(semi_major), area, confidence);
        blade.semi_minor_px = static_cast<float>(semi_minor);
        // RotatedRect::angle 指的是 width 轴的方向；width < height 时长轴要转 90°
        double angle_deg = ellipse.angle;
        if (ellipse.size.width < ellipse.size.height) angle_deg += 90.0;
        blade.angle_rad = static_cast<float>(angle_deg * CV_PI / 180.0);
        blade.hole_count = enclosed_holes(raw_mask, ellipse);

        if (ellipse_fill >= cfg_.min_ellipse_fill && solidity >= cfg_.min_solidity) {
            out.push_back(blade);
        } else if (confidence > fallback_score) {
            fallback = blade;
            fallback_score = confidence;
        }
    }
//没有满足条件的就返回fallback
    if (allow_fallback && out.empty() && fallback_score >= 0.0) {
        out.push_back(fallback);
    }
}




//近距离时这里的ring_converge用来筛选中间的R标，目标周围只有一条亮着的线，R>1
double BladeDetector::ring_coverage(const cv::Mat& mask, const cv::Point2f& center,
                                    double radius) {
    if (radius < 2.0) return 0.0;
    const int samples = 360;
    int total = 0;
    int lit = 0;
    for (int i = 0; i < samples; ++i) {
        const double a = 2.0 * CV_PI * i / samples;
        const int x = cvRound(center.x + radius * std::cos(a));
        const int y = cvRound(center.y + radius * std::sin(a));
        if (x < 1 || y < 1 || x >= mask.cols - 1 || y >= mask.rows - 1) continue;
        ++total;
        // 3×3 邻域任一为亮就算命中，抗锯齿和细线抖动
        bool hit = false;
        for (int dy = -1; dy <= 1 && !hit; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (mask.at<std::uint8_t>(y + dy, x + dx) != 0) {
                    hit = true;
                    break;
                }
            }
        }
        if (hit) ++lit;
    }
    if (total == 0) return 0.0;
    return static_cast<double>(lit) / total;
}//其实同样会被面积上下限晒掉

// namespace {

// // 找圆心：灯臂是五条长亮线，Hough 找直线，RANSAC 找“被最多直线穿过”的点。
// // 半分辨率跑，8 m 处灯臂仍有 ~40 px；扇叶边缘只有 ~15 px，会被长度闸滤掉。
// void find_rune_geometry(const cv::Mat& mask, const std::vector<Blade>& blades,
//                         RuneGeometry& geo) {
//     geo = RuneGeometry{};
//     if (mask.empty() || blades.empty()) return;

//     // 灯臂 + 中心 R 标：把扇叶区域挖掉，剩下的 5 条臂是旋转对称的，
//     // 它们的像素质心就是圆心。比 Hough 找直线稳，也不受两条边的偏移影响。
//     cv::Mat arms = mask.clone();
//     for (const auto& b : blades) {
//         const int rx = std::max(2, cvRound(b.radius_px * 1.15));
//         const int ry = std::max(2, cvRound(b.semi_minor_px * 1.15));
//         cv::ellipse(arms, b.center, cv::Size(rx, ry),
//                     static_cast<double>(b.angle_rad) * 180.0 / CV_PI, 0.0, 360.0,
//                     cv::Scalar(0), cv::FILLED);
//     }

//     cv::Mat labels, stats, cents;
//     const int n = cv::connectedComponentsWithStats(arms, labels, stats, cents, 8);
//     double sx = 0.0, sy = 0.0, sw = 0.0;
//     for (int i = 1; i < n; ++i) {
//         const int area = stats.at<int>(i, cv::CC_STAT_AREA);
//         if (area < 30) continue;
//         sx += cents.at<double>(i, 0) * area;
//         sy += cents.at<double>(i, 1) * area;
//         sw += area;
//     }
//     if (sw < 100.0) return;

//     const cv::Point2f center(static_cast<float>(sx / sw),
//                              static_cast<float>(sy / sw));
//     double sum_r = 0.0;
//     for (const auto& b : blades) {
//         sum_r += cv::norm(b.center - center);
//     }
//     const float radius = static_cast<float>(sum_r / blades.size());
//     if (radius < 15.0f || radius > 300.0f) return;

//     const Blade& b = blades.front();  // 已按面积排序，取最大那块算相位
//     const cv::Point2f d = b.center - center;
//     geo.valid = true;
//     geo.center = center;
//     geo.radius_px = radius;
//     geo.phase_rad = std::atan2(d.y, d.x);
// }

// }

std::vector<Blade> BladeDetector::detect(const cv::Mat& bgr, RuneGeometry* geometry) {
    std::vector<Blade> out;
    if (bgr.empty()) return out;

    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    // 掩码
    cv::Mat mask = cv::Mat::zeros(hsv.size(), CV_8U);
    if (cfg_.color != RuneColor::Blue) {
        cv::Mat mask1, mask2;
        cv::inRange(hsv, cfg_.red1_low, cfg_.red1_high, mask1);
        cv::inRange(hsv, cfg_.red2_low, cfg_.red2_high, mask2);
        cv::bitwise_or(mask1, mask2, mask);
    }
    if (cfg_.color != RuneColor::Red) {
        cv::Mat mask_b;
        cv::inRange(hsv, cfg_.blue_low, cfg_.blue_high, mask_b);
        cv::bitwise_or(mask, mask_b, mask);
    }

    const double img_area = static_cast<double>(bgr.cols) * bgr.rows;
    const int alt_k = cfg_.morph_px == kAltKernelPx ? 5 : kAltKernelPx;

    collect(mask, cfg_.morph_px, img_area, cfg_.allow_shape_fallback, out);

    if (out.size() < static_cast<size_t>(std::max(1, cfg_.max_blades))) {
        std::vector<Blade> extra;
        collect(mask, alt_k, img_area, false, extra);
        for (const auto& b : extra) {
            bool duplicated = false;
            for (const auto& a : out) {
                if (cv::norm(a.center - b.center) <
                    0.7 * static_cast<double>(std::max(a.radius_px, b.radius_px))) {
                    duplicated = true;
                    break;
                }
            }
            if (!duplicated) out.push_back(b);
        }
    }

    // 剔除机关中心的 R 标
    if (cfg_.hub_ring_coverage_max > 0.0) {
        out.erase(std::remove_if(out.begin(), out.end(),
                                 [&](const Blade& b) {
                                     // 太小的候选跳过这道判据：2r 的圆会压在灯臂上，
                                     // 覆盖率必然超标，会把远处真靶误杀。
                                     if (static_cast<double>(b.radius_px) <
                                         cfg_.hub_ring_min_radius_px) {
                                         return false;
                                     }
                                     return ring_coverage(mask, b.center,
                                                          b.radius_px * cfg_.hub_ring_scale) >
                                            cfg_.hub_ring_coverage_max;
                                 }),
                  out.end());
    }

    // 同帧尺寸一致性：扔掉比"本帧最大那块"小得多的候选
    if (cfg_.min_radius_ratio_to_max > 0.0) {
        double max_r = 0.0;
        for (const auto& b : out) max_r = std::max(max_r, static_cast<double>(b.radius_px));
        if (max_r > 0.0) {
            const double radius_floor = max_r * cfg_.min_radius_ratio_to_max;
            out.erase(std::remove_if(out.begin(), out.end(),
                                     [radius_floor](const Blade& b) {
                                         return static_cast<double>(b.radius_px) <
                                                radius_floor;
                                     }),
                      out.end());
        }
    }

    // 面积大的优先
    std::sort(out.begin(), out.end(), [](const Blade& a, const Blade& b) {
        return a.area_px > b.area_px;
    });
    if (out.size() > static_cast<size_t>(std::max(1, cfg_.max_blades))) {
        out.resize(static_cast<size_t>(cfg_.max_blades));
    }
    for (size_t i = 0; i < out.size(); ++i) {
        out[i].id = static_cast<uint8_t>(i);
    }

    // if (geometry) find_rune_geometry(mask, out, *geometry);
    return out;
}

}  // namespace rune
