#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>

#include "rune_exam/msg/blade_keypoints.hpp"
#include "rune_exam/msg/target_info.hpp"
#include "rune_min/blade_tracker.hpp"
#include "rune_min/pnp.hpp"
#include "rune_min/track_selector.hpp"

class RuneSolverNode : public rclcpp::Node {
public:
    RuneSolverNode() : Node("rune_solver") {
        //declare_parameter<>()在参数服务器上注册一个参数类型string,名字叫keypoints_topic，先代一个默认值，后面会被覆盖
        keypoints_topic_ = declare_parameter<std::string>(
            "keypoints_topic", "/rune/blade_keypoints");
        camera_info_topic_ = declare_parameter<std::string>(
            "camera_info_topic", "/rm_energy/camera/camera_info");
        target_topic_ = declare_parameter<std::string>(
            "target_topic", "/rune/target_info");
        feature_diameter_m_ = declare_parameter<double>("feature_diameter_m", 0.308);
        // 仿真器 D 全零，开不开都一样
        assume_undistorted_ = declare_parameter<bool>("assume_undistorted", false);

        // 收不到 CameraInfo 时的兜底内参；填 0 就必须等 CameraInfo。
        // 仿真的内参是 fx=fy=935.307436087, cx=960, cy=540。
        const double fx = declare_parameter<double>("fx", 0.0);
        const double fy = declare_parameter<double>("fy", 0.0);
        const double cx = declare_parameter<double>("cx", 0.0);
        const double cy = declare_parameter<double>("cy", 0.0);
        if (fx > 0.0 && fy > 0.0) {
            K_ = (cv::Mat_<double>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
            D_ = cv::Mat::zeros(5, 1, CV_64F);
            RCLCPP_INFO(get_logger(), "用配置里的兜底内参 fx=%.3f cx=%.1f", fx, cx);
        }



//把参数变成对象，把话题接到回调上

        //跟踪器
        rune::BladeTrackerConfig tracker_cfg;
        tracker_cfg.max_missed = static_cast<int>(
            declare_parameter<int>("track_max_missed", tracker_cfg.max_missed));
        tracker_cfg.confirm_hits = static_cast<int>(
            declare_parameter<int>("track_confirm_hits", tracker_cfg.confirm_hits));
        tracker_cfg.velocity_alpha =
            declare_parameter<double>("track_velocity_alpha", tracker_cfg.velocity_alpha);
        tracker_cfg.gate_radius_ratio =
            declare_parameter<double>("track_gate_ratio", tracker_cfg.gate_radius_ratio);
        tracker_ = rune::BladeTracker(tracker_cfg);

        //选靶子
        rune::TrackSelectConfig select_cfg;
        select_cfg.pending_min_holes =
            static_cast<int>(declare_parameter<int>("pending_min_holes", 2));
        select_cfg.switch_after_missed = static_cast<int>(
            declare_parameter<int>("switch_after_missed", select_cfg.switch_after_missed));
        select_cfg.activate_confirm = static_cast<int>(
            declare_parameter<int>("activate_confirm", select_cfg.activate_confirm));
        selector_ = rune::TrackSelector(select_cfg);

        pub_target_ = create_publisher<rune_exam::msg::TargetInfo>(target_topic_, 10);


        //订阅
        sub_camera_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic_, rclcpp::SensorDataQoS(),
            std::bind(&RuneSolverNode::on_camera_info, this, std::placeholders::_1));

        sub_keypoints_ = create_subscription<rune_exam::msg::BladeKeypoints>(
            keypoints_topic_, 10,
            std::bind(&RuneSolverNode::on_keypoints, this, std::placeholders::_1));


        RCLCPP_INFO(get_logger(),
                    "solver: %s -> %s (feature ⌀%.0f mm, 跟踪 max_missed=%d, 换靶丢失 %d 帧)",
                    keypoints_topic_.c_str(), target_topic_.c_str(),
                    feature_diameter_m_ * 1000.0, tracker_cfg.max_missed,
                    select_cfg.switch_after_missed);
                    //[rune_solver] solver: /rune/blade_keypoints -> /rune/target_info (feature ⌀308 mm, 跟踪 max_missed=30, 换靶丢失 5 帧)
    }

private:
    void on_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        // CameraInfo.k = [fx 0 cx; 0 fy cy; 0 0 1]
        if (msg->k[0] <= 0.0 || msg->k[4] <= 0.0) return;

        K_ = (cv::Mat_<double>(3, 3) << msg->k[0], msg->k[1], msg->k[2],
                                       msg->k[3], msg->k[4], msg->k[5],
                                       msg->k[6], msg->k[7], msg->k[8]);
        D_ = cv::Mat::zeros(static_cast<int>(msg->d.size()), 1, CV_64F);
        for (size_t i = 0; i < msg->d.size(); ++i) {
            D_.at<double>(static_cast<int>(i)) = msg->d[i];
        }
        // 关键点已经过识别端去畸变时不再用pnp投影
        if (assume_undistorted_) D_ = cv::Mat::zeros(5, 1, CV_64F);
    }

    void on_keypoints(const rune_exam::msg::BladeKeypoints::SharedPtr msg) {
        rune_exam::msg::TargetInfo out;
        out.header = msg->header;
        out.valid = false;

        // 时间戳：直接沿用图像时间，设限必须单调若失败则回退回上一帧+1/30
        double stamp = rclcpp::Time(msg->header.stamp).seconds();
        if (!(stamp > 0.0)) stamp = last_stamp_ + 1.0 / 30.0;
        if (!(stamp > last_stamp_)) stamp = last_stamp_ + 1.0 / 30.0;
        last_stamp_ = stamp;

  //把从recognition_node传来的ros信息转为结构体里的信息发给blade_detect blade_track pnp track_select
        std::vector<rune::Blade> detected;
        detected.reserve(msg->blades.size());
        for (const auto& blade : msg->blades) {
            rune::Blade data;
            data.valid = true;
            data.id = blade.id;
            data.confidence = blade.confidence;
            data.center = cv::Point2f(blade.center.x, blade.center.y);//tracker pnp
            data.radius_px = blade.radius_px;//tracker pnp
            data.area_px = blade.area_px;//blade
            data.semi_minor_px = blade.semi_minor_px;//pnp
            data.angle_rad = blade.angle_rad;//pnp
            data.hole_count = blade.hole_count;//blade
            detected.push_back(data);
        }



    //输入本地的观测update预测 关联 更新 滑行 新建 删除 限制数目,tracks_更新，每个靶子有一个id
        tracker_.update(detected, stamp);
    //当前已知的所有目标
        const std::vector<rune::BladeTrack>& tracks = tracker_.tracks();
//TrackerSelectedResults里所有的字段都来自TrackSelector
        const rune::TrackSelectResult selected = selector_.select(tracks);
        //track返回nullptr或missed则不发目标，正常则送去pnp
        const rune::BladeTrack* track =
            selected.index >= 0 ? &tracks[static_cast<std::size_t>(selected.index)] : nullptr;




        if (selected.all_activated) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "所有跟踪目标都被判为已激活，退化为全部可打"
                                 "（检查 pending_min_holes / hole_count）");
        }
        if (track != nullptr) out.id = track->id;
        if (selected.switched) {
            // 换靶本来应该是“打中了/真丢了”才发生；频繁出现说明跟踪参数要调。
            RCLCPP_INFO(get_logger(), "换靶 -> id=%d (%s)",
                        static_cast<int>(selected.track_id),
                        selected.target_activated ? "已激活" : "丢失/首次锁定");
        }

        // // ---- 传统法：选中靶相对机关圆心的相位/角速度（先只打日志）----
        // // 圆心和扇叶在同一帧图像里，相机自己的转动对两者影响相同，相减后抵消，
        // // 所以 φ 只含机关自己的旋转。匀速档 φ 是直线，ω 恒定。
        // if (track != nullptr && track->missed == 0 && msg->rune_valid &&
        //     msg->rune_radius_px > 1.0f) {
        //     const double dx = track->center.x - msg->rune_center.x;
        //     const double dy = track->center.y - msg->rune_center.y;
        //     const double ph = std::atan2(dy, dx);
        //     constexpr double kPi = 3.14159265358979323846;
        //     if (!phase_valid_ || track->id != phase_id_ || stamp - phase_t_ > 0.3) {
        //         phase_valid_ = true;
        //         phase_id_ = track->id;
        //         phase_ = ph;
        //         phase_omega_ = 0.0;
        //     } else {
        //         double d = ph - phase_;
        //         while (d > kPi) d -= 2.0 * kPi;
        //         while (d < -kPi) d += 2.0 * kPi;
        //         const double dt = stamp - phase_t_;
        //         if (dt > 1e-3) {
        //             phase_ += d;
        //             phase_omega_ = 0.7 * phase_omega_ + 0.3 * (d / dt);
        //         }
        //     }
        //     phase_t_ = stamp;
        //     RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        //                          "rune id=%d phase=%.3f omega=%.2f r=%.0f",
        //                          static_cast<int>(track->id), phase_, phase_omega_,
        //                          msg->rune_radius_px);
        // }


        //没内参、没目标、目标在coast、PnP解失败了，这四种情况都不能发目标，不然云台会受到一个错误的数值
        if (K_.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "还没收到 CameraInfo，也没有配置兜底内参，先发无效目标");
        } else if (track == nullptr || track->radius_px < 1.0f) {
            RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "本帧没有亮靶");
        } else if (track->missed > 0) {
            // 短暂丢帧：不拿旧半径硬解 PnP，本帧不发目标。
            // 检测层要保证不丢帧，这里不做“沿用上一帧”的掩盖。
            RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                                  "track %d 丢失（missed=%d），本帧不更新位置",
                                  static_cast<int>(track->id), track->missed);
        } else {
            // 平面 PnP：椭圆的长短轴四个端点 <-> 靶面圆周上互相垂直的四点，
            // 解出来的 tvec 就是**靶心在相机系下的位置**（米）。
            const auto pnp = rune::solve_disc_pnp(
                track->center, static_cast<double>(track->radius_px),
                static_cast<double>(track->semi_minor_px),
                static_cast<double>(track->angle_rad), feature_diameter_m_, K_, D_);

            if (!pnp.valid) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "PnP 失败（重投影 %.2f px），本帧不发目标",
                                     pnp.reproj_error);
            } else {
                const double x = pnp.tvec[0];
                const double y = pnp.tvec[1];
                const double z = pnp.tvec[2];

                out.valid = true;
                // 只在“待击打”时请求开火。全部目标都被判成已激活时，
                // hole_count 已经不可信（远距离/阈值没标定），退化为允许开火，
                // 否则会出现“有目标但永远不打”的死局。
                out.fire = selected.pending || selected.all_activated;
                out.pixel.x = track->center.x;
                out.pixel.y = track->center.y;
                out.pixel.z = 0.0;
                out.distance_m = static_cast<float>(std::sqrt(x * x + y * y + z * z));
                out.depth_m = static_cast<float>(z);
                out.position_cam.x = x;
                out.position_cam.y = y;
                out.position_cam.z = z;

                RCLCPP_INFO_THROTTLE(
                    get_logger(), *get_clock(), 1000,
                    "id=%d holes=%d pixel=(%.1f, %.1f) dist=%.3f m depth=%.3f m "
                    "cam=(%.3f, %.3f, %.3f) reproj=%.2f px",
                    static_cast<int>(out.id), track->hole_count, out.pixel.x, out.pixel.y,
                    out.distance_m, out.depth_m, x, y, z, pnp.reproj_error);
            }
        }
//填写TargetInfo发到/rune/target_info，云台gimbal_node拆解其中的数据
        pub_target_->publish(out);
    }

    std::string keypoints_topic_;
    std::string camera_info_topic_;
    std::string target_topic_;
    double feature_diameter_m_ = 0.308;
    bool assume_undistorted_ = false;

    // ---- 跟踪 + 选靶
    rune::BladeTracker tracker_;
    rune::TrackSelector selector_;

    // 相机内参（PnP 用）。启动时用 fx/fy/cx/cy 参数
    cv::Mat K_;
    cv::Mat D_;

    // 上一帧图像时间戳：保证喂给跟踪器的 dt 单调、不为负。
    double last_stamp_ = 0.0;

    // // 传统法：选中靶相对机关圆心的相位（累计、已解卷绕）和角速度。
    // bool phase_valid_ = false;
    // int phase_id_ = -1;
    // double phase_ = 0.0;
    // double phase_omega_ = 0.0;
    // double phase_t_ = 0.0;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_;
    rclcpp::Subscription<rune_exam::msg::BladeKeypoints>::SharedPtr sub_keypoints_;
    rclcpp::Publisher<rune_exam::msg::TargetInfo>::SharedPtr pub_target_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RuneSolverNode>());
    rclcpp::shutdown();
    return 0;
}
