// 云台节点：/rune/target_info → 预测 + 弹道解算 → /rm_energy/gimbal/command
// 约定：指令角相对初始姿态；姿态按图像 stamp 反查（oracle 优先）；
// 弹道补重力；前馈只用绝对视线角的 ω；开火看预测误差而不是当前偏差。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "rm_energy_interfaces/msg/gimbal_command.hpp"
#include "rune_exam/msg/target_info.hpp"
#include "rune_min/angle_tracker.hpp"
#include "rune_min/pose_source.hpp"

class RuneGimbalNode : public rclcpp::Node {
public:
    // 订阅 /rm_energy/debug/labels 和 /rune/target_info，把图像的相对角换算成绝对方向，
    // 最终输出 /rm_energy/gimbal/command 驱动仿真器转动和击发
    RuneGimbalNode() : Node("rune_gimbal") {
        target_topic_ = declare_parameter<std::string>(
            "target_topic", "/rune/target_info");
        command_topic_ = declare_parameter<std::string>(
            "command_topic", "/rm_energy/gimbal/command");

        // ---- 弹道参数 ----
        muzzle_speed_ = declare_parameter<double>("muzzle_speed_mps", 20.0);
        gravity_ = declare_parameter<double>("gravity_mps2", 9.81);
        extra_latency_ = declare_parameter<double>("extra_latency_s", 0.05);  // 额外提前量，系统延迟
        gimbal_lag_s_ = declare_parameter<double>("gimbal_lag_s", 0.03);
        gimbal_lag_s_ = std::clamp(gimbal_lag_s_, 0.0, 0.1);
        control_rate_hz_ = declare_parameter<double>("control_rate_hz", 50.0);  // 控制频率

        // ---- 姿态来源 ----
        // oracle：labels 给真实云台角（首选）；cmd：自己发过的指令 + gimbal_lag 兜底。
        pose_source_ = declare_parameter<std::string>("pose_source", "auto");
        use_oracle_ = (pose_source_ != "command");
        labels_topic_ = declare_parameter<std::string>("labels_topic", "/rm_energy/debug/labels");
        // oracle 样本比"现在"旧超过这个时长就弃用
        pose_timeout_s_ = declare_parameter<double>("pose_timeout_s", 0.10);

        // 零指令时的相机初始仰角；绝对俯仰 = 初始仰角 + 指令
        initial_elevation_ = declare_parameter<double>("initial_elevation_rad", 0.044078972191);
        // 枪口相对相机：右 0.08 / 下 0.06 / 前 0.25
        muzzle_x_ = declare_parameter<double>("muzzle_x_m", 0.08);
        muzzle_y_ = declare_parameter<double>("muzzle_y_m", 0.06);
        muzzle_z_ = declare_parameter<double>("muzzle_z_m", 0.25);
        enable_ballistic_ = declare_parameter<bool>("enable_ballistic", true);
        // ---- 云台限位（public_data.yaml）----这里是指令角，真实可达要再加初始仰角2.53
        yaw_min_ = declare_parameter<double>("yaw_min_rad", -0.698131700798);  // -40
        yaw_max_ = declare_parameter<double>("yaw_max_rad", 0.698131700798);   // 40
        pitch_min_ = declare_parameter<double>("pitch_min_rad", -0.209439510239);  // -12
        pitch_max_ = declare_parameter<double>("pitch_max_rad", 0.314159265359);   // 18

        // ---- 瞄准/开火 ----
        fire_miss_m_ = declare_parameter<double>("fire_miss_m", 0.10);  // 落点偏差，若过大则不开火；yaml 0.06
        fire_interval_ = declare_parameter<double>("fire_interval_s", 0.5);  // 两发最小间隔
        fire_settle_s_ = declare_parameter<double>("fire_settle_s", 0.3);  // 找到靶后稳定这么久再打；yaml 0.8
        enable_fire_ = declare_parameter<bool>("enable_fire", true);
        // 连续 5 拍都在门限内才开火（50 Hz 下 5 拍 = 100 ms），避免扫过门限那一瞬
        fire_confirm_ticks_ = static_cast<int>(
            declare_parameter<int>("fire_confirm_ticks", 5));
        // 估计器模式稳定这么久才允许开火；模式乱跳说明模型不自洽
        fire_mode_hold_s_ = declare_parameter<double>("fire_mode_hold_s", 0.3);
        // ---- 开火模型质量门：确定"模型预测准"才开火，而不是只"对准了预测点" ----
        // miss<fire_miss_m 只保证云台对准了模型给的预测点，不保证预测点=真靶心。
        // 换靶后圆拟合相位/圆心还没收敛时，预测点偏了，miss 照样能 <0.06，打出去就偏。
        // 所以匀速档要求圆拟合达到开火级质量（点数够 + 残差/尾部创新小），
        // 静止档没有圆可拟合，用"角速度长期接近 0"代替。
        fire_require_model_ = declare_parameter<bool>("fire_require_model", true);
        fire_circle_max_residual_ =
            declare_parameter<double>("fire_circle_max_residual", 0.05);
        fire_circle_max_tail_ =
            declare_parameter<double>("fire_circle_max_tail", 0.05);
        fire_min_circle_samples_ =
            declare_parameter<int>("fire_min_circle_samples", 10);
        fire_static_rate_radps_ =
            declare_parameter<double>("fire_static_rate_radps", 0.05);
        fire_static_hold_frames_ =
            declare_parameter<int>("fire_static_hold_frames", 15);
        // "命中时刻的期望角"超出云台限位就不开火
        limit_guard_ = declare_parameter<bool>("fire_limit_guard", true);
        // 目标跳变时把"一帧甩过去"变成"平滑转过去"
        max_yaw_rate_ = declare_parameter<double>("max_yaw_rate_radps", 2.0);
        max_pitch_rate_ = declare_parameter<double>("max_pitch_rate_radps", 2.0);

        // ---- α-β（绝对视线角）----
        const double angle_alpha = declare_parameter<double>("angle_filter_alpha", 0.5);  // 信测量 50%、信预测 50%
        abs_yaw_track_ = rune::AngleTracker(angle_alpha);
        abs_pitch_track_ = rune::AngleTracker(angle_alpha);

        // ---- 圆拟合的闸门 ----
        rune_arm_radius_m_ = declare_parameter<double>("rune_arm_radius_m", 0.7);
        // 半径闸：|r − 0.7/距离| < 55% 才采纳这次拟合；填 0 = 关掉这道闸
        circle_radius_tol_ =
            declare_parameter<double>("circle_radius_tol_ratio", 0.55);
        // 弧长闸：相位跨度必须 > 0.6 radetect
        circle_min_phase_span_rad_ =
            declare_parameter<double>("circle_min_phase_span_rad", 0.6);
        // 残差闸：径向残差 / 半径 < 0.20
        circle_max_rel_residual_ =
            declare_parameter<double>("circle_max_rel_residual", 0.20);
        // 尾部创新闸：最后 5 点相对模型的平均相位偏差 < 0.12
        circle_max_tail_innovation_ =
            declare_parameter<double>("circle_max_tail_innovation", 0.12);
        // 连续这么多帧模型解释不了新点就清空重建
        circle_reset_after_reject_ = static_cast<int>(
            declare_parameter<int>("circle_reset_after_reject", 10));
        // 比例增益；kp=10 会在靶心附近形成极限环（静止也晃）
        aim_kp_ = declare_parameter<double>("aim_kp", 4.0);
        // 死区 0.004 rad ≈ 5 m 处 2 cm，远小于命中半径
        aim_deadband_rad_ =
            declare_parameter<double>("aim_deadband_rad", 0.004);

        pub_ = create_publisher<rm_energy_interfaces::msg::GimbalCommand>(
            command_topic_, 10);
        sub_ = create_subscription<rune_exam::msg::TargetInfo>(
            target_topic_, 10,
            std::bind(&RuneGimbalNode::on_target, this, std::placeholders::_1));

        // 真实姿态话题，仿真器默认发布
        if (use_oracle_) {
            sub_labels_ = create_subscription<std_msgs::msg::String>(
                labels_topic_, rclcpp::SensorDataQoS(),
                std::bind(&RuneGimbalNode::on_labels, this, std::placeholders::_1));
        }

        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_hz_)));  // 周期 0.02 秒
        timer_ = create_wall_timer(period, std::bind(&RuneGimbalNode::tick, this));  // 系统时间

        // 输入话题 / 输出话题 / 弹速 / 额外延迟 / 控制频率 / 速率限幅 / 姿态源 / 开火确认
        RCLCPP_INFO(get_logger(),
                    "gimbal: %s -> %s (v=%.1f m/s, extra_latency=%.3f s, %.0f Hz, "
                    "rate=%.1f/%.1f rad/s, pose=%s, 开火确认=%d 拍)",
                    target_topic_.c_str(), command_topic_.c_str(), muzzle_speed_,
                    extra_latency_, control_rate_hz_, max_yaw_rate_, max_pitch_rate_,
                    use_oracle_ ? "oracle(+指令历史兜底)" : "仅指令历史",
                    fire_confirm_ticks_);
    }

private:
    double now_s() const { return this->now().seconds(); }

    // ---- 姿态：oracle（真实）优先，cmd（指令历史 + lag）兜底 ----
    bool pose_at(double t, double& yaw, double& pitch, bool& from_oracle) {
        if (use_oracle_) {
            double y = 0.0, p = 0.0;
            if (oracle_pose_.at(t, y, p)) {
                yaw = y;
                pitch = p;
                from_oracle = true;
                return true;
            }
        }
        from_oracle = false;
        if (cmd_pose_.at(t - gimbal_lag_s_, yaw, pitch)) {
            return true;
        }
        return false;
    }

    // 当前姿态：真实姿态是零阶保持的，取最新样本即可
    bool pose_now(double t, double& yaw, double& pitch, bool& from_oracle,
                  double* t_sample = nullptr) {
        if (use_oracle_) {
            rune::PoseSample s;
            if (oracle_pose_.latest(s) && (t - s.t) <= pose_timeout_s_) {
                yaw = s.yaw;
                pitch = s.pitch;
                from_oracle = true;
                if (t_sample) *t_sample = s.t;
                return true;
            }
        }
        from_oracle = false;
        const double t_q = t - gimbal_lag_s_;
        if (t_sample) *t_sample = t_q;
        return cmd_pose_.at(t_q, yaw, pitch);
    }

    void on_labels(const std_msgs::msg::String::SharedPtr msg) {
        rune::PoseSample s;
        if (!rune::parse_pose_labels(msg->data, s)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "labels 里没有 stamp_ns/yaw_rad/pitch_rad，姿态退回指令历史");
            return;
        }
        oracle_pose_.add(s.t, s.yaw, s.pitch);
    }

    // 低伸弹道解：tanθ = (v² − √(v⁴ − g(gR² + 2hv²))) / (gR)
    static double solve_elevation(double R, double h, double v, double g,
                                  bool& reachable) {
        if (R < 1e-3) {
            reachable = false;
            return 0.0;
        }
        const double v2 = v * v;
        const double disc = v2 * v2 - g * (g * R * R + 2.0 * h * v2);
        if (disc < 0.0) {
            reachable = false;
            return 0.7853981633974483;  // 打不到：给 45°，让云台先抬起来
        }
        reachable = true;
        const double u = (v2 - std::sqrt(disc)) / (g * R);
        return std::atan(u);
    }

    void on_target(const rune_exam::msg::TargetInfo::SharedPtr msg) {
        // 顺序：校验目标 → 换靶检测 → 量测 → 预测 → 控制量
        fire_request_ = msg->fire;  // 解算节点选出了待击打的靶面
        if (!msg->valid) {
            // 不清零角速度：断帧时清零会让云台每动一下就被刹车
            have_target_ = false;
            return;
        }

        const Eigen::Vector3d p(msg->position_cam.x, msg->position_cam.y,
                                msg->position_cam.z);
        if (!p.allFinite() || p.z() <= 0.05) {
            have_target_ = false;
            return;
        }

        // ---- 时间轴：图像采集时刻 vs 现在 ----
        // epoch stamp 时 now - stamp 就是真实图像延迟；非 epoch 无法对齐。
        const double t_now = now_s();
        double t_img = rclcpp::Time(msg->header.stamp).seconds();
        double meas_latency = 0.0;
        const bool epoch_stamp = t_img > 1.0e9;  // 2001 年以后的 epoch 秒
        bool stamp_ok = true;
        if (epoch_stamp) {
            meas_latency = t_now - t_img;
            stamp_ok = std::isfinite(meas_latency) &&
                       meas_latency > -0.02 && meas_latency < 0.5;
        } else {
            if (t_img > 0.0) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 5000,
                    "图像 stamp 不是 epoch（%.3fs），时间轴无法对齐", t_img);
            }
            stamp_ok = false;
        }

        // 采集那一刻的云台姿态（p 是采集时相机系下的位置，必须配对）
        double yaw_at = cmd_yaw_;
        double pitch_at = cmd_pitch_;
        bool at_from_oracle = false;
        bool have_at = false;
        if (stamp_ok) have_at = pose_at(t_img, yaw_at, pitch_at, at_from_oracle);
        const bool compensate = stamp_ok && have_at;

        // 当前云台姿态
        double yaw_now_cam = cmd_yaw_;
        double pitch_now_cam = cmd_pitch_;
        bool now_from_oracle = false;
        if (stamp_ok) (void)pose_now(t_now, yaw_now_cam, pitch_now_cam, now_from_oracle);

        // ---- 换目标检测 ----
        // 只有位置真的跳了（> 0.3 m）才重置估计器；断帧重建出的假 id 不重置。
        if (!have_track_id_ || msg->id != track_id_) {
            track_id_ = msg->id;
            have_track_id_ = true;
            const bool jumped = !have_last_p_ || (p - last_p_).norm() > 0.3;
            if (jumped) {
                abs_yaw_track_.reset();
                abs_pitch_track_.reset();
                yaw_fit_.reset();
                pitch_fit_.reset();
                // 做法 A：换靶只换相位，圆心（机关中心）和半径（物理值）都不变。
                // 用物理角半径 0.7/距离 锁定半径，复用换靶前圆心，只清相位。
                // 锁定失败（换靶前圆还没拟合出来）才退回硬 reset。
                const double r0 =
                    rune_arm_radius_m_ / std::max(0.5, std::hypot(p.x(), p.z()));
                if (!abs_circle_.soft_reset(r0)) {
                    abs_circle_.reset();
                }
                yaw_rate_cmd_ = 0.0;
                pitch_rate_cmd_ = 0.0;
                last_meas_t_ = 0.0;
                // 只有真换靶才重新计瞄准稳定时间
                target_since_s_ = t_now;
            }
        }
        if (!have_last_p_) target_since_s_ = t_now;  // 第一次拿到目标也要等收敛
        last_p_ = p;
        have_last_p_ = true;

        // 两次量测的间隔，用图像时间戳，不用本机时间
        const double dt_meas =
            (last_meas_t_ > 0.0) ? (t_img - last_meas_t_) : (1.0 / 30.0);
        last_meas_t_ = t_img;

        const Eigen::Vector3d muzzle(muzzle_x_, muzzle_y_, muzzle_z_);

        // ---- 当前相对误差（不假设"指令角等于真实姿态"）----
        // d 按采集时的仰角转到世界系（x右, y上, z前）；必须用 pitch_at。
        const double phi_at = initial_elevation_ + pitch_at;
        const Eigen::Vector3d d = p - muzzle;
        const double py = -d.y() * std::cos(phi_at) + d.z() * std::sin(phi_at);
        const double pz = d.y() * std::sin(phi_at) + d.z() * std::cos(phi_at);
        const double range = std::hypot(d.x(), pz);

        const double e_yaw = std::atan2(d.x(), pz);  // 目标在光轴右侧为正

        bool reachable = true;
        const double theta = enable_ballistic_
                                 ? solve_elevation(range, py, muzzle_speed_, gravity_,
                                                   reachable)
                                 : std::atan2(py, range);
        const double e_pitch = theta - phi_at;  // 采集时光轴还差多少

        // 绝对视线角 = 采集时云台角 + 采集时相对误差（两项同一时刻）
        const double theta_yaw_meas = yaw_at + e_yaw;
        const double theta_pitch_meas = initial_elevation_ + pitch_at + e_pitch;

        // 世界系绝对方向：相机自己的转动被 yaw_at/pitch_at 抵消
        const double abs_pitch_geo = std::atan2(py, range);
        if (compensate) {
            abs_circle_.add(t_img, theta_yaw_meas, abs_pitch_geo);
        }

        // 飞行时间按弹道算：t = R / (v·cosθ)
        const double flight_time =
            range / std::max(1e-3, muzzle_speed_ * std::cos(theta));
        // 上限必须覆盖最远可打距离：20 m/s 下 0.6 s 只对应 12 m。
        // 超过 12 m 时原实现会直接把提前量截断，匀速档按 0.73 m/s 算，
        // 每少 0.1 s 就少给 7.3 cm，表现为"稳定打后"而不是随机误差。
        // 1.5 s 对应 30 m，实际场景够用。
        const double lead = std::clamp(flight_time + extra_latency_, 0.0, 1.5);

        // ---- 预测：从"采集时"推到"命中时" ----
        // 三级估计器，样本够就覆盖前一个：
        //   ① α-β 始终运行兜底  ② 二次拟合 0.3 s 后  ③ 圆拟合（几何+质量闸全过）
        struct Aim {
            double yaw_now = 0.0;
            double pitch_now = 0.0;
            double yaw_aim = 0.0;
            double pitch_aim = 0.0;
            double yaw_rate = 0.0;  // 当前角速度估计（日志用）
            double pitch_rate = 0.0;
            double yaw_ff = 0.0;  // 前馈（控制用）
            double pitch_ff = 0.0;
            int mode = 0;  // 0=α-β, 1=二次拟合, 2=圆拟合
        };
        Aim aim;
        aim.yaw_now = e_yaw;
        aim.pitch_now = e_pitch;
        aim.yaw_aim = e_yaw;
        aim.pitch_aim = e_pitch;
        fire_model_ok_ = false;  // 这一帧默认不达标，圆拟合收敛/目标静止时再置 true

        if (compensate) {
            // ① α-β：绝对视线角 [角度, 角速度]
            abs_yaw_track_.update(theta_yaw_meas, dt_meas);
            abs_pitch_track_.update(theta_pitch_meas, dt_meas);
            aim.yaw_rate = std::clamp(abs_yaw_track_.rate(), -2.5, 2.5);
            aim.pitch_rate = std::clamp(abs_pitch_track_.rate(), -2.5, 2.5);

            const double dt_to_now = std::max(0.0, t_now - t_img);
            aim.yaw_now = abs_yaw_track_.predict(dt_to_now) - yaw_now_cam;
            aim.pitch_now = abs_pitch_track_.predict(dt_to_now) -
                            (initial_elevation_ + pitch_now_cam);
            aim.yaw_aim = aim.yaw_now + aim.yaw_rate * lead;
            aim.pitch_aim = aim.pitch_now + aim.pitch_rate * lead;
            aim.yaw_ff = aim.yaw_rate;
            aim.pitch_ff = aim.pitch_rate;

            // ③ 绝对方向圆拟合：靶心绕机关中心转，拟合圆后按 φ(t) 外推到命中时刻
            // 锁定半径随距离刷新（距离变了角半径也要变，否则锁定值过期）
            abs_circle_.set_radius(rune_arm_radius_m_ / std::max(0.5, range));
            auto abs_c = abs_circle_.fit();
            const double expected_r = rune_arm_radius_m_ / std::max(0.5, range);
            // 三道几何闸（填 0 = 不检查）
            const bool span_ok =
                abs_circle_.locked() ||
                circle_min_phase_span_rad_ <= 0.0 ||
                std::fabs(abs_c.phase_span) > circle_min_phase_span_rad_;
            const bool radius_ok =
                circle_radius_tol_ <= 0.0 ||
                std::fabs(abs_c.r - expected_r) < circle_radius_tol_ * expected_r;
            // 两道质量闸：残差杀噪声团，尾部创新杀换档污染
            const bool residual_ok =
                abs_c.rel_residual < circle_max_rel_residual_;
            const bool tail_ok =
                abs_c.tail_innovation < circle_max_tail_innovation_;
            const bool geom_ok = abs_c.ok && span_ok && radius_ok &&
                                 residual_ok && tail_ok;
            if (geom_ok) {
                circle_reject_frames_ = 0;
                const double t_impact = t_now + lead;
                double yaw_p = 0.0, pitch_p = 0.0;
                if (abs_c.predict(t_impact, yaw_p, pitch_p)) {
                    aim.mode = 2;
                    // 圆拟合达到"开火级"质量：点数够（换靶后 3~7 点也能拟合成功，
                    // 但圆心/相位还没收敛）+ 残差/尾部创新远小于采纳闸（0.20/0.12）
                    fire_model_ok_ =
                        static_cast<int>(abs_circle_.size()) >=
                            fire_min_circle_samples_ &&
                        abs_c.rel_residual < fire_circle_max_residual_ &&
                        abs_c.tail_innovation < fire_circle_max_tail_;
                    aim.yaw_aim = yaw_p - yaw_now_cam;
                    const double height = range * std::tan(pitch_p);
                    bool reachable_p = true;
                    const double theta_p =
                        enable_ballistic_
                            ? solve_elevation(range, height, muzzle_speed_, gravity_,
                                              reachable_p)
                            : std::atan2(height, range);
                    aim.pitch_aim = theta_p - (initial_elevation_ + pitch_now_cam);
                    const double phase_p =
                        abs_c.phase_ref + abs_c.omega * (t_impact - abs_c.t_ref);
                    aim.yaw_ff = -abs_c.r * abs_c.omega * std::sin(phase_p);
                    aim.pitch_ff = abs_c.r * abs_c.omega * std::cos(phase_p);
                    aim.yaw_now = theta_yaw_meas - yaw_now_cam;
                    aim.pitch_now = theta - (initial_elevation_ + pitch_now_cam);
                    // 圆拟合的 ω 是绕机关中心的角速度，不是 yaw/pitch 各自的导数
                    aim.yaw_rate = aim.yaw_ff;
                    aim.pitch_rate = aim.pitch_ff;
                    RCLCPP_INFO_THROTTLE(
                        get_logger(), *get_clock(), 1000,
                        "abs yaw_c=%.3f pitch_c=%.3f r=%.4f(期望%.4f) "
                        "omega=%.2f span=%.2f res=%.3f tail=%.3f n=%zu",
                        abs_c.cx, abs_c.cy, abs_c.r, expected_r, abs_c.omega,
                        abs_c.phase_span, abs_c.rel_residual, abs_c.tail_innovation,
                        abs_circle_.size());
                }
            } else if (abs_c.ok) {
                // 只有"数据够但模型解释不了新点"才算缓存过期；弧不够长不算
                const bool data_ok = span_ok && radius_ok;
                const bool quality_ok = residual_ok && tail_ok;
                if (data_ok && !quality_ok) {
                    if (++circle_reject_frames_ >= circle_reset_after_reject_) {
                        circle_reject_frames_ = 0;
                        abs_circle_.reset();
                        yaw_fit_.reset();
                        pitch_fit_.reset();
                        RCLCPP_INFO(get_logger(),
                                    "圆拟合连续 %d 帧没通过质量闸，清空重建"
                                    "（r=%.4f res=%.3f tail=%.3f）",
                                    circle_reset_after_reject_, abs_c.r,
                                    abs_c.rel_residual, abs_c.tail_innovation);
                    }
                } else {
                    circle_reject_frames_ = 0;
                }
                RCLCPP_INFO_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "圆拟合被闸拦掉 r=%.4f 期望 %.4f span=%.2f res=%.3f tail=%.3f"
                    "（span=%d radius=%d res=%d tail=%d）—— 退回 α-β/二次拟合",
                    abs_c.r, expected_r, abs_c.phase_span, abs_c.rel_residual,
                    abs_c.tail_innovation, span_ok ? 1 : 0, radius_ok ? 1 : 0,
                    residual_ok ? 1 : 0, tail_ok ? 1 : 0);
            } else {
                circle_reject_frames_ = 0;
            }
        } else {
            // 时间轴不可用：清估计，退回纯比例
            abs_yaw_track_.reset();
            abs_pitch_track_.reset();
            yaw_fit_.reset();
            pitch_fit_.reset();
            abs_circle_.reset();
        }

        // 静止档判定：没有圆拟合（aim.mode != 2）且角速度长期接近 0，也算"模型可信"。
        if (compensate && aim.mode != 2 &&
            std::fabs(aim.yaw_rate) < fire_static_rate_radps_ &&
            std::fabs(aim.pitch_rate) < fire_static_rate_radps_) {
            if (fire_static_frames_ < 1000000) ++fire_static_frames_;
        } else {
            fire_static_frames_ = 0;
        }
        if (fire_static_frames_ >= fire_static_hold_frames_) {
            fire_model_ok_ = true;
        }

        // ---- 控制量：前馈跟上目标转速，比例只修残差 ----
        yaw_rate_cmd_ = aim.yaw_ff + aim_kp_ * aim.yaw_aim;
        pitch_rate_cmd_ = aim.pitch_ff + aim_kp_ * aim.pitch_aim;
        if (std::fabs(aim.yaw_aim) < aim_deadband_rad_) yaw_rate_cmd_ = aim.yaw_ff;
        if (std::fabs(aim.pitch_aim) < aim_deadband_rad_)
            pitch_rate_cmd_ = aim.pitch_ff;

        // 模型自洽的偏差（看不见模型本身的偏置，真正的判据在 tick 里）
        aim_miss_m_ = range * std::sin(
            std::min(1.0, std::hypot(aim.yaw_aim, aim.pitch_aim)));

        // ---- 留给 tick()：这一帧"云台应该指到哪" ----
        desired_yaw_cmd_ = yaw_now_cam + aim.yaw_aim;
        desired_pitch_cmd_ = pitch_now_cam + aim.pitch_aim;
        desired_yaw_rate_ = aim.yaw_ff;
        desired_pitch_rate_ = aim.pitch_ff;
        desired_t_ = t_now;
        desired_range_m_ = range;
        have_desired_ = compensate;

        // 模式稳定了多久：模式每帧跳就说明模型不自洽
        if (aim.mode != last_aim_mode_) {
            last_aim_mode_ = aim.mode;
            aim_mode_since_s_ = t_now;
        }

        have_target_ = true;

        // 每秒一次现场排查日志
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                             "aim p=(%.2f,%.2f,%.2f) lat=%.0fms e=(%.3f,%.3f) "
                             "now=(%.3f,%.3f) w=(%.2f,%.2f) lead=%.3fs fit=%d "
                             "rate=(%.2f,%.2f) miss=%.3f comp=%d pose=%s",
                             p.x(), p.y(), p.z(), meas_latency * 1e3,
                             e_yaw, e_pitch,
                             aim.yaw_now, aim.pitch_now,
                             aim.yaw_rate, aim.pitch_rate, lead,
                             aim.mode,
                             yaw_rate_cmd_, pitch_rate_cmd_, aim_miss_m_,
                             compensate ? 1 : 0,
                             at_from_oracle ? "oracle" : "cmd");

        if (!compensate) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "指令历史不够老（刚启动）或时间轴不可用（lat=%.1f ms）："
                "本帧退回纯比例、不加前馈。稍等几帧即可恢复",
                meas_latency * 1e3);
        }
    }

    void tick() {
        const double t = now_s();

        // 控制 50 Hz，与图像 30 Hz 解耦：这里把角速度积分成角度并限幅
        double dt = (last_tick_s_ > 0.0) ? (t - last_tick_s_)
                                         : (1.0 / std::max(1.0, control_rate_hz_));
        last_tick_s_ = t;
        if (!(dt > 1e-4) || dt > 0.10) {
            dt = 1.0 / std::max(1.0, control_rate_hz_);
        }

        // 目标短暂丢失：速率缓慢衰减（约 0.2 s 归零），避免急停
        if (!have_target_) {
            yaw_rate_cmd_ *= 0.85;
            pitch_rate_cmd_ *= 0.85;
            if (std::fabs(yaw_rate_cmd_) < 1e-3) yaw_rate_cmd_ = 0.0;
            if (std::fabs(pitch_rate_cmd_) < 1e-3) pitch_rate_cmd_ = 0.0;
            fire_confirm_seen_ = 0;
        }

        const double yaw_rate =
            std::clamp(yaw_rate_cmd_, -max_yaw_rate_, max_yaw_rate_);
        const double pitch_rate =
            std::clamp(pitch_rate_cmd_, -max_pitch_rate_, max_pitch_rate_);
        cmd_yaw_ = std::clamp(cmd_yaw_ + yaw_rate * dt, yaw_min_, yaw_max_);
        cmd_pitch_ =
            std::clamp(cmd_pitch_ + pitch_rate * dt, pitch_min_, pitch_max_);

        // 记录这一刻发出的指令，供按 stamp 反查
        cmd_pose_.add(t, cmd_yaw_, cmd_pitch_);

        // ---- 开火判据：用**当前真实姿态**重算，而不是模型自己的 miss ----
        double miss_for_fire = aim_miss_m_;
        if (have_desired_) {
            double yaw_now = cmd_yaw_;
            double pitch_now = cmd_pitch_;
            bool pose_ok = false;
            if (pose_now(t, yaw_now, pitch_now, pose_ok)) {
                // desired_yaw_cmd_ 已经是"命中时刻"的期望角（aim 里含 lead 外推），
                // 云台要追的就是这个方向，直接和当前真实姿态比即可，不要再外推。
                // 之前用 t_pose−desired_t_ 对齐时 desired_t_ 设成了 t_now，但 desired
                // 对应的是 t_now+lead，少减一个 lead 就把期望角多推了 lead·ω，
                // 弹着点沿靶运动方向系统性偏（超前还是落后看 ω 符号）。
                constexpr double kPi = 3.14159265358979323846;
                double dy = desired_yaw_cmd_ - yaw_now;
                while (dy > kPi) dy -= 2.0 * kPi;
                while (dy < -kPi) dy += 2.0 * kPi;
                const double dp = desired_pitch_cmd_ - pitch_now;
                miss_now_m_ = desired_range_m_ * std::hypot(dy, dp);
                miss_for_fire = miss_now_m_;
            }
        }

        // 连续 N 拍都在门限内才算真对准（收敛过程中扫过门限不算）
        if (miss_for_fire < fire_miss_m_) {
            if (fire_confirm_seen_ < 1000000) ++fire_confirm_seen_;
        } else {
            fire_confirm_seen_ = 0;
        }

        rm_energy_interfaces::msg::GimbalCommand cmd;
        cmd.header.stamp = now();
        cmd.header.frame_id = "gimbal";
        cmd.yaw_rad = cmd_yaw_;
        cmd.pitch_rad = cmd_pitch_;
        cmd.fire = false;

        const bool mode_settled =
            (t - aim_mode_since_s_) > fire_mode_hold_s_;

        // ---- 限位保护 ----
        // 区分两件事：饱和（现在顶在限位，但目标可达）允许开火；
        // 超限（连"该指到的角"都在限位外）物理上打不到，按住不开火。
        constexpr double kLimitEps = 1.0e-3;  // 1 mrad ≈ 7 mm @7 m
        const bool aim_out_of_range =
            limit_guard_ && have_desired_ &&
            (desired_pitch_cmd_ > pitch_max_ + kLimitEps ||
             desired_pitch_cmd_ < pitch_min_ - kLimitEps ||
             desired_yaw_cmd_ > yaw_max_ + kLimitEps ||
             desired_yaw_cmd_ < yaw_min_ - kLimitEps);

        if (aim_out_of_range) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "目标超出云台限位，按住不开火：期望 pitch=%.3f（限位 %.3f~%.3f）"
                " yaw=%.3f（%.3f~%.3f）",
                desired_pitch_cmd_, pitch_min_, pitch_max_,
                desired_yaw_cmd_, yaw_min_, yaw_max_);
        }

        // 开火：误差达标 + 模型可信 + 连续确认 + 模式稳定 + 过了 settle/间隔 + 没超限
        if (enable_fire_ && have_target_ && fire_request_ &&
            !aim_out_of_range &&
            (!fire_require_model_ || fire_model_ok_) &&
            miss_for_fire < fire_miss_m_ &&
            fire_confirm_seen_ >= fire_confirm_ticks_ &&
            mode_settled &&
            (t - target_since_s_) > fire_settle_s_ &&
            (t - last_fire_s_) > fire_interval_) {
            // 上升沿触发：这一拍 true，下一拍自动回 false
            cmd.fire = true;
            last_fire_s_ = t;
            fire_confirm_seen_ = 0;  // 打完重新累计
            RCLCPP_INFO(get_logger(),
                        "FIRE  miss=%.3f(模型 %.3f) yaw=%.3f pitch=%.3f",
                        miss_now_m_, aim_miss_m_, cmd.yaw_rad, cmd.pitch_rad);
        } else if (enable_fire_ && have_target_ && fire_request_) {
            // 该打没打：把每道闸的实际值/要求值都打出来
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "HOLD miss=%.3f/%.3f confirm=%d/%d mode=%.2f/%.2f "
                "settle=%.2f/%.2f gap=%.2f/%.2f model=%d lim=%d",
                miss_for_fire, fire_miss_m_, fire_confirm_seen_,
                fire_confirm_ticks_, t - aim_mode_since_s_, fire_mode_hold_s_,
                t - target_since_s_, fire_settle_s_,
                t - last_fire_s_, fire_interval_,
                fire_model_ok_ ? 1 : 0, aim_out_of_range ? 1 : 0);
        }

        pub_->publish(cmd);
    }

    std::string target_topic_;
    std::string command_topic_;

    // 弹道
    double muzzle_speed_ = 20.0;
    double gravity_ = 9.81;
    double extra_latency_ = 0.05;
    double gimbal_lag_s_ = 0.03;
    double control_rate_hz_ = 50.0;

    // 几何
    double initial_elevation_ = 0.044078972191;
    double muzzle_x_ = 0.08;
    double muzzle_y_ = 0.06;
    double muzzle_z_ = 0.25;
    bool enable_ballistic_ = true;

    double yaw_min_ = -0.698;
    double yaw_max_ = 0.698;
    double pitch_min_ = -0.209;
    double pitch_max_ = 0.314;

    // 开火
    double fire_miss_m_ = 0.10;
    double fire_interval_ = 0.5;
    double fire_settle_s_ = 0.3;
    bool enable_fire_ = true;

    // 执行限幅
    double max_yaw_rate_ = 2.0;    // rad/s
    double max_pitch_rate_ = 2.0;  // rad/s

    // 闭环 PD：前馈跟上转速，比例回中心
    double aim_kp_ = 4.0;              // 1/s
    double aim_deadband_rad_ = 0.004;  // rad

    // 绝对视线角 α-β（yaw / pitch 各一个）
    rune::AngleTracker abs_yaw_track_;
    rune::AngleTracker abs_pitch_track_;
    double last_meas_t_ = 0.0;

    // 绝对角的短窗二次拟合 + 圆拟合
    rune::QuadraticFit yaw_fit_;
    rune::QuadraticFit pitch_fit_;
    rune::CircleFit2D abs_circle_;

    // 圆拟合的闸门
    double rune_arm_radius_m_ = 0.7;        // m
    double circle_radius_tol_ = 0.55;       // 允许 ±55% 偏差
    double circle_min_phase_span_rad_ = 0.6;  // 圆弧至少走过的相位
    double circle_max_rel_residual_ = 0.20;
    double circle_max_tail_innovation_ = 0.12;
    int circle_reset_after_reject_ = 10;
    int circle_reject_frames_ = 0;          // 连续被质量闸拒的帧数

    // 姿态源
    std::string pose_source_;
    std::string labels_topic_;
    bool use_oracle_ = true;
    double pose_timeout_s_ = 0.10;
    rune::PoseHistory cmd_pose_{256, 6.0};     // 50 Hz 下约 5 s
    rune::PoseHistory oracle_pose_{256, 3.0};

    // 云台指令（相对初始姿态）
    double cmd_yaw_ = 0.0;
    double cmd_pitch_ = 0.0;
    double yaw_rate_cmd_ = 0.0;    // rad/s，tick 里积分
    double pitch_rate_cmd_ = 0.0;
    double last_tick_s_ = 0.0;

    // 跟踪 id：变了就说明换目标
    std::uint16_t track_id_ = 0;
    bool have_track_id_ = false;
    Eigen::Vector3d last_p_ = Eigen::Vector3d::Zero();
    bool have_last_p_ = false;

    double aim_miss_m_ = 1e9;
    bool have_target_ = false;
    bool fire_request_ = false;
    double target_since_s_ = 0.0;
    double last_fire_s_ = -1e9;

    // 开火判据（tick 里用当前姿态重算）
    double desired_yaw_cmd_ = 0.0;    // 期望的绝对指令角
    double desired_pitch_cmd_ = 0.0;
    double desired_yaw_rate_ = 0.0;   // 期望角的导数，外推到开火时刻
    double desired_pitch_rate_ = 0.0;
    double desired_t_ = 0.0;
    double desired_range_m_ = 0.0;
    bool have_desired_ = false;
    double miss_now_m_ = 1e9;         // 用当前姿态算出来的真实偏差
    int fire_confirm_ticks_ = 5;
    int fire_confirm_seen_ = 0;
    double fire_mode_hold_s_ = 0.3;
    bool limit_guard_ = true;
    double aim_mode_since_s_ = 0.0;
    int last_aim_mode_ = -1;

    // 开火模型质量门
    bool fire_require_model_ = true;
    double fire_circle_max_residual_ = 0.05;
    double fire_circle_max_tail_ = 0.05;
    int fire_min_circle_samples_ = 10;
    double fire_static_rate_radps_ = 0.05;
    int fire_static_hold_frames_ = 15;
    bool fire_model_ok_ = false;   // 这一帧模型质量够开火（圆拟合收敛 或 目标静止）
    int fire_static_frames_ = 0;   // 连续"几乎不转"帧数（静止档）

    rclcpp::Subscription<rune_exam::msg::TargetInfo>::SharedPtr sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_labels_;
    rclcpp::Publisher<rm_energy_interfaces::msg::GimbalCommand>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RuneGimbalNode>());
    rclcpp::shutdown();
    return 0;
}
