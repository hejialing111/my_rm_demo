#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

namespace rune {

// 能量机关阵营颜色。红方灯效是红的、蓝方是蓝的。
// Both = 红蓝都找（默认：仿真器是红的；蓝灯只作为兼容保留）。
enum class RuneColor { Red = 0, Blue = 1, Both = 2 };

// 亮靶检测参数。
//
// 真实仿真画面里靶盘的样子（实测）：**黑底 + 几圈细红线**的同心圆靶纹，挂在灯臂末端。
// 注意不是“红环套一个黑洞”——早期版本按这个假设去找“洞”，结果找到的全是红线之间
// 的缝隙（半径只有 6 px），框歪、闪、误检、漏检全来了。
//
// 现在的做法：用小核闭运算把同心环填成实心圆，再用**椭圆三判据**
// （aspect / ellipse_fill / solidity）把它和 R 标、箭头灯带、地面亮线区分开。
struct BladeDetectConfig {
    // 找哪个颜色的灯效
    RuneColor color = RuneColor::Both;

    // HSV 红色两段。实测靶盘的 H 落在 0~15，S/V 最低到 90/70，
    // 所以阈值要比“纯红”宽一些，否则同心环会被切碎。
    cv::Scalar red1_low{0, 90, 70};
    cv::Scalar red1_high{15, 255, 255};
    cv::Scalar red2_low{165, 90, 70};
    cv::Scalar red2_high{180, 255, 255};

    // HSV 蓝色一段。实测：H 95~125，S 中位 244，V 中位 154。
    cv::Scalar blue_low{95, 120, 50};
    cv::Scalar blue_high{130, 255, 255};

    // 面积过滤（相对整幅图，天然与分辨率无关）
    double min_area_ratio = 0.0002;
    double max_area_ratio = 0.05;

    // 形态学核：固定像素，**不要按图幅自动放大**。
    //
    // 扫过 3/5/7/9/11（正视 892x489 + 斜视 1714x957 + 考核视频 1440x1080）：
    //   3、5  → 样张都只框到真靶，无假框
    //   7     → 斜视时多出中心假框
    //   9、11 → 斜视时漏掉真靶，正视时多出假框
    // 原因是核一大就把靶盘和旁边结构粘成一片（斜视漏检），
    // 同时把中心 R 区糊成一个紧凑圆块（R 误检）。两个问题是同一个原因。
    //
    // 考核视频上逐帧统计：3 漏 0 帧，5 漏 1 帧，7 漏 4 帧。
    // 所以主用 3；一个都没找够时自动用 5 再补一遍（见 detect()）。
    int morph_px = 3;

    // ---- 椭圆三判据（对相机倾角不敏感）----
    // 斜视时靶盘被压成椭圆：圆度 = 短轴/长轴，倾 60° 就掉到 0.5 而被误杀。
    // 椭圆拟合的三个量对“压扁”免疫：椭圆面积恒为 πab。
    //
    //   aspect        = 短轴/长轴        太扁 = 灯带/地面亮线（实测 0.12~0.14）
    //   ellipse_fill  = 轮廓面积/(πab)   不规则形状会偏低
    //   solidity      = 面积/凸包面积    字母 R 有凹口，实测 0.78；靶盘 0.96~0.97
    double min_aspect = 0.30;
    double min_ellipse_fill = 0.85;
    double min_solidity = 0.90;

    // 半径下限（像素）。再小就只是噪点或红线缝隙。
    double min_radius_px = 4.0;

    // 同帧尺寸一致性：候选半径 < 本帧最大半径 × 这个系数，就当误检丢掉。
    //
    // 为什么需要：规则书里 R 标是 ⌀106、靶面是 ⌀300，比值 0.353。实测同一帧
    // R 标 r≈14.9 px、真靶面 r≈46.2 px，比值 0.32。而 3×3 闭运算会把字母 R
    // 的**内孔填死**，于是 solidity 这道专门杀 "R" 的判据反而失效（实测 R 标
    // 面积 498 px²，轻松过了 311 px² 的面积闸），结果就是中间那个 R 被框出来。
    //
    // 它是**相对**阈值（分母是本帧最大块），近/远整体缩放时不受影响；
    // 唯一会误杀的窗口是"近距离 + 大倾斜"造成的透视差：
    //   (d − 0.7)/(d + 0.7) < 0.55  →  d < 2.4 m
    // 设成 0 可以关掉。
    double min_radius_ratio_to_max = 0.55;

    // ---- 剔除机关中心的 R 标：圆周覆盖率 ----
    //
    // R 标在机关中心，5 条臂从它向外辐射；靶面在每条臂的**末端**，只有它自己
    // 那一条臂穿过。所以在半径 `hub_ring_scale × r` 的圆周上统计亮点占比：
    //   靶面（臂末端）：0.03 ~ 0.04
    //   R 标（5 臂汇聚）：0.24 ~ 0.32
    // 4~8 倍差距，和尺寸、距离都无关，所以近处远处都成立。
    //
    // 实测：这条把 208/209/253 三帧"只有 R 标被检出"的误检也清掉了
    // （那种帧里 R 是唯一候选，尺寸比例闸管不到它）。
    double hub_ring_scale = 2.0;        // 采样圆周半径 = 该倍数 × 候选半径
    double hub_ring_coverage_max = 0.15; // 超过这个覆盖率判为 R 标并丢弃；<=0 关闭
    // 半径小于这个值就不做圆周覆盖率判据。
    // 原因：小目标（远处靶只有 ~19 px）时 2r 的圆几乎全压在灯臂上，覆盖率必然超阈值，
    // 会把真靶间歇性杀掉 —— 2026-11 仿真实测：靶半径 18.8 px 时检测时有时无。
    // 此时 R 标半径只有 ~6.6 px、面积 < 面积闸，本来也进不来，跳过这道判据是安全的。
    double hub_ring_min_radius_px = 25.0;

    // 形状兜底：本帧一个合格候选都没有时，交出"形状不达标但不太扁"的那个。
    //
    // 为什么要它：远处小目标（半径 ~19 px）会和灯臂时粘时离，粘连的那帧
    // ellipse_fill/solidity 会掉到阈值以下，直接丢掉就变成隔帧丢检
    // （仿真实测 /rune/target_info 在 true/false 之间反复跳）。宁可测量略偏也不丢帧。
    //
    // 但**看视频/调阈值时必须关掉**：视频里没有"丢一帧就扣分"的压力，而一帧里只要
    // 有一块亮斑（地面反光、灯带、灰蓝色块），兜底就会把它当成一块靶发出去 ——
    // 表现就是"根本不是椭圆、轮廓都不闭合的也被检测到了"。设 false 之后，
    // 只有真正通过 ellipse_fill + solidity 的候选才会被发布。
    bool allow_shape_fallback = true;

    // 仿真同一时刻最多两块靶亮
    int max_blades = 2;
};

// 一块亮起的靶面。坐标全部是**图像坐标**（像素，原点左上）。
struct Blade {
    uint8_t id = 0;
    bool valid = false;
    float confidence = 0.0f;

    cv::Point2f center{0.0f, 0.0f};
    float radius_px = 0.0f;  // 靶纹（同心圆）的外半径 = 椭圆半长轴
    double area_px = 0.0;

    // 拟合椭圆的完整描述，PnP 要用。
    // angle_rad 已归一化成**长轴**方向（fitEllipse 的 angle 指 width 轴，
    // width < height 时要 +90°），别直接拿 RotatedRect.angle 用。
    float semi_minor_px = 0.0f;
    float angle_rad = 0.0f;

    // 靶面内部被红色围住的“暗区”个数（原始掩码）。
    //
    // 规则书结构：未击打的靶面是几圈同心环，环间每道缝隙都是一个闭合暗区；
    // 击打成功后整块亮成实心，内部暗区只剩一个/没有。所以这个数量和状态相关。
    int hole_count = 0;

    // 5 个关键点：[0]=中心，[1]=上，[2]=右，[3]=下，[4]=左
    std::array<cv::Point2f, 5> keypoints{};
};

// 机关圆心/相位（传统视觉用）。
// 圆心 = 五条灯臂的交点（Hough 直线 + RANSAC）；相位 = 最大扇叶相对圆心的角度。
// 坐标在**检测图**（view）坐标系里，ROI/缩放时调用方自己换算。
struct RuneGeometry {
    bool valid = false;
    cv::Point2f center{0.0f, 0.0f};
    float radius_px = 0.0f;  // 扇叶中心到圆心的像素距离
    float phase_rad = 0.0f;  // atan2(dy, dx)，图像坐标 y 向下
};

// 亮靶检测器：红色掩码 -> 闭运算填实 -> 按圆度挑出靶盘。
class BladeDetector {
public:
    explicit BladeDetector(const BladeDetectConfig& config = {});

    // 返回按面积从大到小排好序的亮靶（最多 max_blades 个）。
    // geometry 非空时，顺带给出机关圆心/相位（传统视觉的输入）。
    std::vector<Blade> detect(const cv::Mat& bgr, RuneGeometry* geometry = nullptr);

private:
    // 用指定核大小跑一遍「闭运算 -> 找轮廓 -> 椭圆三判据」，结果追加到 out。
    // allow_fallback：本遍一个合格候选都没有时，是否交出“形状不达标但不太扁”的候选。
    // 只在主核那一遍打开；备用核再开会在“只有一块真靶”时凭空补出第二块假靶。
    void collect(const cv::Mat& raw_mask, int kernel_px, double img_area,
                 bool allow_fallback, std::vector<Blade>& out) const;

    // 在半径 radius 的圆周上统计"亮点占整圈的比例"。
    // 靶面（臂末端）~0.03，R 标（5 臂汇聚）~0.3，用来看候选是不是机关中心。
    static double ring_coverage(const cv::Mat& mask, const cv::Point2f& center,
                                double radius);

    BladeDetectConfig cfg_;
};

}  // namespace rune
