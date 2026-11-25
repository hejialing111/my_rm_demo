#include "rune_min/pnp.hpp"

#include <cmath>
#include <vector>

#include <opencv2/calib3d.hpp>

namespace rune {
namespace {

// 盘面法线在相机 z 轴上的分量，就是旋转矩阵 R 的 (2,2) 项。
//
// 相机朝 +z 看，所以"能看见正面的靶面"其外法线必须背向相机：nz < 0。
// 这一项用来在 IPPE 的两个解里挑出物理上成立的那个。
double disc_normal_z(const cv::Vec3d& rvec) {
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    return R.at<double>(2, 2);
}

}  // namespace
//solce_planar_pnp为solve_disc_pnp准备必须的平面上的四个点
PlanarPnPResult solve_planar_pnp( const std::vector<cv::Point3f>& object_points,
                                  const std::vector<cv::Point2f>& image_points,
                                  const cv::Mat& K,
                                  const cv::Mat& D) 
    {
    PlanarPnPResult result;
    if (object_points.size() < 4 || object_points.size() != image_points.size()) {
        return result;
    }
//输出旋转向量和旋转矩阵
    std::vector<cv::Vec3d> rvecs, tvecs;
    try {
        //这里的solvePnPGeneric算出平面上的每一个解并通过重投影算出各自的误差
        cv::solvePnPGeneric(object_points, image_points, K, D,
                            rvecs, tvecs, false, cv::SOLVEPNP_IPPE);
    } catch (const cv::Exception&) {
        return result;
    }

    int best = -1;
    double best_err = 1e9;

    for (size_t i = 0; i < rvecs.size(); ++i) {
        if (!std::isfinite(tvecs[i][0]) ||
            !std::isfinite(tvecs[i][1]) ||
            !std::isfinite(tvecs[i][2]) ||
            tvecs[i][2] <= 0.0) {
            continue;
        }

        std::vector<cv::Point2f> projected;
        //projectPoints算出理论上的重投影值
        cv::projectPoints(object_points, rvecs[i], tvecs[i], K, D, projected);

        double err = 0.0;
        for (size_t k = 0; k < projected.size(); ++k) {
            err += cv::norm(projected[k] - image_points[k]);
        }
        err /= static_cast<double>(projected.size());

        if (err < best_err) {
            best_err = err;
            best = static_cast<int>(i);
        }
    }

    if (best < 0) return result;

    result.valid = best_err < 5.0;
    result.rvec = rvecs[best];
    result.tvec = tvecs[best];
    result.reproj_error = best_err;
    return result;
}

PlanarPnPResult solve_disc_pnp(const cv::Point2f& center_px,
                               double semi_major_px,
                               double semi_minor_px,
                               double angle_rad,
                               double diameter_m,
                               const cv::Mat& K,
                               const cv::Mat& D) {
    const double radius = 0.5 * diameter_m;
    const double c = std::cos(angle_rad);
    const double s = std::sin(angle_rad);
    const auto fr = [](double v) { return static_cast<float>(v); };

    // 物体系：圆盘中心为原点，盘面 z = 0，圆周半径 radius。
    const std::vector<cv::Point3f> object_points = {
        {fr(radius), 0.0f, 0.0f},
        {fr(-radius), 0.0f, 0.0f},
        {0.0f, fr(radius), 0.0f},
        {0.0f, fr(-radius), 0.0f},
    };

    // 图像系：椭圆长轴/短轴的四个端点，正好对应上面四个圆周点。
    const std::vector<cv::Point2f> image_points = {
        {fr(center_px.x + semi_major_px * c), fr(center_px.y + semi_major_px * s)},
        {fr(center_px.x - semi_major_px * c), fr(center_px.y - semi_major_px * s)},
        {fr(center_px.x - semi_minor_px * s), fr(center_px.y + semi_minor_px * c)},
        {fr(center_px.x + semi_minor_px * s), fr(center_px.y - semi_minor_px * c)},
    };

    PlanarPnPResult result =
        solve_planar_pnp(object_points, image_points, K, D);
    if (!result.valid) return result;

    // 圆盘正反两面是"长相一样"的：把物体绕自身 x 轴翻 180°，四个圆周点的
    // 集合不变、投影也完全不变。所以 IPPE 的两个解常常**都**是背面朝前，
    // 重投影误差分不开（实测两组解 nz 都是 +0.9，真值应该是 -0.9）。
    //
    // 看得见靶面就说明正面朝相机，nz 必为负。这里显式翻一次：
    //   R' = R · Rx(180°)  →  nz 取反，投影不变。
    // 只影响盘面朝向（rvec），靶心位置 tvec 不变。
    if (disc_normal_z(result.rvec) > 0.0) {
        cv::Mat R;
        cv::Rodrigues(result.rvec, R);
        const cv::Mat flip_x =
            (cv::Mat_<double>(3, 3) << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0);
        cv::Rodrigues(R * flip_x, result.rvec);
    }
    return result;
}

}  // namespace rune
