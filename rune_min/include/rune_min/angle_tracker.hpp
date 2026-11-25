#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <vector>

namespace rune {
//Angleracker把一串带噪声的角度测量值变成角速度和外推模型
class AngleTracker {
public:
    explicit AngleTracker(double alpha = 0.6, double max_rate = 6.0)
        : alpha_(clamp_alpha(alpha)),//0.05-1
          beta_(2.0 * (2.0 - clamp_alpha(alpha)) -
                4.0 * std::sqrt(1.0 - clamp_alpha(alpha))),
          max_rate_(max_rate) {}
    void reset() {
        x_ = 0.0;//当前角度估计
        v_ = 0.0;//当前角速度估计
        init_ = false;
    }
    bool initialized() const { return init_; }
    double angle() const { return x_; }
    double rate() const { return v_; }


    void update(double meas, double dt) {//传入测量值和时间差
        if (!(dt > 1e-4)) dt = 1.0 / 30.0;
        if (dt > 0.30) dt = 0.30;  // 时间戳异常时别一步跨太大

        if (!init_) {
            x_ = meas;
            v_ = 0.0;
            init_ = true;
            return;
        }

        const double pred = x_ + v_ * dt;//预测
        const double r = meas - pred;//新息

        x_ = pred + alpha_ * r;//角度更新
        v_ = std::clamp(v_ + beta_ * r / dt, -max_rate_, max_rate_);//速度更新，有物理上限
    }

    // 从现在起 lead_s 秒后的角度：θ + ω·t
    double predict(double lead_s) const {
        if (!init_) return x_;
        return x_ + v_ * std::max(0.0, lead_s);
    }

private:
    static double clamp_alpha(double a) {
        if (!(a > 0.05)) return 0.05;
        if (a > 1.0) return 1.0;
        return a;
    }

    double alpha_;
    double beta_;
    double max_rate_;

    double x_ = 0.0;
    double v_ = 0.0;
    bool init_ = false;
};





//用最近一小段的时间和角度的点拟合成一条抛物线。用来计算未来某时刻的角度是多少
class QuadraticFit {
public:
    struct Result {
        bool ok = false;
        double a = 0.0;
        double b = 0.0;//角速度
        double c = 0.0;//角加速度
        double tm = 0.0;  //窗口时间均值

        double eval(double t) const {//预测t时的角度
            const double u = t - tm;
            return a + b * u + c * u * u;
        }
        double rate(double t) const {//预测t时的角速度
            const double u = t - tm;
            return b + 2.0 * c * u;
        }
    };

    explicit QuadraticFit(std::size_t max_samples = 16, double max_age_s = 0.8)
        : max_samples_(max_samples), max_age_(max_age_s) {}

    void reset() { buf_.clear(); }

    void add(double t, double y) {
        buf_.push_back(Sample{t, y});
        while (buf_.size() > max_samples_) buf_.pop_front();
        // 时间戳回退/跳变时不至于把整个窗口拖坏
        while (!buf_.empty() && (buf_.back().t - buf_.front().t) > max_age_) {
            buf_.pop_front();
        }
    }

    Result fit() const {
        Result r;
        // 只有 5 个点（0.12 s）时，二次项几乎是被噪声拟合出来的：
        // c 可能大到几 rad/s²，外推 0.3 s 会把瞄准点甩出去，
        // 表现就是换靶后“追几步”。等窗口够长再启用。
        if (buf_.size() < 9) return r;
        const double span = buf_.back().t - buf_.front().t;
        if (!(span > 0.3)) return r;

        double tm = 0.0;
        for (const auto& s : buf_) tm += s.t;
        tm /= static_cast<double>(buf_.size());

        // 以 tm 为中心，u 的一、三次项之和为 0，方程退化成 3 个独立式子
        double s0 = 0.0, s2 = 0.0, s4 = 0.0;
        double b0 = 0.0, b1 = 0.0, b2 = 0.0;
        for (const auto& s : buf_) {
            const double u = s.t - tm;
            const double u2 = u * u;
            s0 += 1.0;
            s2 += u2;
            s4 += u2 * u2;
            b0 += s.y;
            b1 += u * s.y;
            b2 += u2 * s.y;
        }
        const double det = s0 * s4 - s2 * s2;
        if (!(std::fabs(det) > 1e-12) || !(s2 > 1e-12)) return r;

        r.a = (b0 * s4 - b2 * s2) / det;
        r.b = b1 / s2;
        r.c = (s0 * b2 - s2 * b0) / det;
        // 曲率异常（噪声/跳变）时宁可退回 α-β，也不要拿它外推
        if (!(std::fabs(r.c) < 3.0)) return r;
        r.tm = tm;
        r.ok = true;
        return r;
    }

private:
    struct Sample {
        double t = 0.0;
        double y = 0.0;
    };

    std::deque<Sample> buf_;
    std::size_t max_samples_;
    double max_age_;
};




//已知一串带时间戳的点,用这组参数预测圆心半径和相位角速度
class CircleFit2D {
public:
    struct Result {
        bool ok = false;//拟合成功
        double cx = 0.0;
        double cy = 0.0;
        double r = 0.0;
        double phase_ref = 0.0;//参考时刻的相位
        double t_ref = 0.0;
        double omega = 0.0;
        double phase_span = 0.0;//相机走过的弧度

        double rel_residual = 0.0;//拟合残差/半径，真圆0.02，静止噪声团0.4
        double tail_innovation = 0.0;//相对模型的平均偏差

        bool predict(double t, double& x, double& y) const {
            if (!ok) return false;
            const double ph = phase_ref + omega * (t - t_ref);
            x = cx + r * std::cos(ph);
            y = cy + r * std::sin(ph);
            return true;
        }
    };

    explicit CircleFit2D(std::size_t max_samples = 60, double max_age_s = 2.0)
        : max_samples_(max_samples), max_age_(max_age_s) {}

    void reset() {
        buf_.clear();
        locked_ = false;
    }
    std::size_t size() const { return buf_.size(); }
    bool locked() const { return locked_; }

    // 换靶时的“软重置”：圆心/半径锁定，只清相位数据。
    // r0 是物理角半径（0.7 m / 距离），比上一段拟合的半径可靠得多。
    // 返回 true 表示锁定成功（换靶前有可用圆心），false 表示需退回硬 reset。
    bool soft_reset(double r0) {
        const Result r = fit();
        buf_.clear();
        if (r.ok && r0 > 0.005) {
            lock_cx_ = r.cx;
            lock_cy_ = r.cy;
            lock_r_ = r0;
            lock_omega_ = r.omega;
            locked_ = true;
        } else {
            locked_ = false;
        }
        return locked_;
    }

    // 更新锁定半径：相机距离变化时角半径（0.7/距离）会变，需随距离刷新，
    // 否则锁定值过期会导致圆心精修和 radius_ok 闸全部失准。
    void set_radius(double r0) {
        if (r0 > 0.005) lock_r_ = r0;
    }

    void add(double t, double x, double y) {
        buf_.push_back(Sample{t, x, y});
        while (buf_.size() > max_samples_) buf_.pop_front();
        while (!buf_.empty() && (buf_.back().t - buf_.front().t) > max_age_) {
            buf_.pop_front();
        }
    }
    Result fit() const {
        Result r;
        const double n = static_cast<double>(buf_.size());
        double cx = 0.0;
        double cy = 0.0;
        double radius = 0.0;

        if (locked_) {
            // 锁定模式：圆心/半径/角速度都锁定，只需确定相位。
            // 换靶只换相位，圆心（机关中心）、半径（物理值）、角速度（匀速档）都不变。
            if (buf_.size() < 3) return r;
            cx = lock_cx_;
            cy = lock_cy_;
            radius = lock_r_;
            // 点数足够时用已知半径迭代精修圆心：换靶后 buf_ 持续积累，
            // 弧长增长后圆心会逐步收敛到真值，比固定 lock 值更准。
            if (buf_.size() >= 8) {
                for (int it = 0; it < 3; ++it) {
                    double JtJ00 = 0.0, JtJ01 = 0.0, JtJ11 = 0.0;
                    double Jte0 = 0.0, Jte1 = 0.0;
                    for (const auto& s : buf_) {
                        const double dx = s.x - cx;
                        const double dy = s.y - cy;
                        const double e = dx * dx + dy * dy - radius * radius;
                        const double jx = -2.0 * dx;
                        const double jy = -2.0 * dy;
                        JtJ00 += jx * jx;
                        JtJ01 += jx * jy;
                        JtJ11 += jy * jy;
                        Jte0 += jx * e;
                        Jte1 += jy * e;
                    }
                    const double det = JtJ00 * JtJ11 - JtJ01 * JtJ01;
                    if (std::fabs(det) < 1e-12) break;
                    const double d0 = (-Jte0 * JtJ11 + Jte1 * JtJ01) / det;
                    const double d1 = (Jte0 * JtJ01 - Jte1 * JtJ00) / det;
                    cx += d0;
                    cy += d1;
                }
            }
            const double omega = lock_omega_;
            const double t_ref = buf_.back().t;

            double prev = 0.0, offset = 0.0;
            bool first = true;
            double ph_first = 0.0;
            double phase_sum = 0.0;
            double sum_sq = 0.0;
            std::vector<double> phs;
            phs.reserve(buf_.size());
            for (const auto& s : buf_) {
                const double dx = s.x - cx;
                const double dy = s.y - cy;
                const double dist = std::sqrt(dx * dx + dy * dy);
                sum_sq += (dist - radius) * (dist - radius);

                double ph = std::atan2(dy, dx);
                if (!first) {
                    while (ph + offset - prev > kPi) offset -= 2.0 * kPi;
                    while (ph + offset - prev < -kPi) offset += 2.0 * kPi;
                } else {
                    ph_first = ph;
                }
                ph += offset;
                prev = ph;
                first = false;
                phs.push_back(ph);
                phase_sum += ph - omega * (s.t - t_ref);
            }
            const double phase_ref = phase_sum / n;

            double tail_sum = 0.0;
            const std::size_t tail_n = std::min<std::size_t>(5, phs.size());
            const std::size_t tail_from = phs.size() - tail_n;
            std::size_t idx = 0;
            for (const auto& s : buf_) {
                if (idx >= tail_from) {
                    tail_sum +=
                        std::fabs(phs[idx] - (phase_ref + omega * (s.t - t_ref)));
                }
                ++idx;
            }

            r.ok = true;
            r.cx = cx; r.cy = cy; r.r = radius;
            r.omega = omega;
            r.t_ref = t_ref;
            r.phase_ref = phase_ref;
            r.phase_span = prev - ph_first;
            r.rel_residual = std::sqrt(sum_sq / n) / std::max(radius, 1e-9);
            r.tail_innovation = tail_sum / static_cast<double>(tail_n);
            return r;
        } else {
            // 完整模式：先拟合圆。至少要有十二个点且时间跨度至少 0.5 s
            if (buf_.size() < 12) return r;
            const double span = buf_.back().t - buf_.front().t;
            if (!(span > 0.5)) return r;

            // 1) 2D 圆：x²+y² = A·x + B·y + C
            double sx = 0, sy = 0, sxx = 0, sxy = 0, syy = 0;
            double bx = 0, by = 0, bc = 0;
            for (const auto& s : buf_) {
                const double c = s.x * s.x + s.y * s.y;
                sx += s.x; sy += s.y; sxx += s.x * s.x; sxy += s.x * s.y;
                syy += s.y * s.y;
                bx += s.x * c; by += s.y * c; bc += c;
            }
            double m[3][3] = {{sxx, sxy, sx}, {sxy, syy, sy}, {sx, sy, n}};
            double rhs[3] = {bx, by, bc};
            double sol[3];
            if (!solve3(m, rhs, sol)) return r;
            cx = 0.5 * sol[0];
            cy = 0.5 * sol[1];
            const double r2 = sol[2] + cx * cx + cy * cy;
            if (!(r2 > 0.0)) return r;
            radius = std::sqrt(r2);
            if (!(radius > 0.005 && radius < 0.5)) return r;  // 角半径 0.3°~28°
        }

        // 2) 相位解卷绕 → φ(t) 线性拟合
        double tm = 0.0;
        for (const auto& s : buf_) tm += s.t;
        tm /= n;

        double prev = 0.0, offset = 0.0;
        bool first = true;
        double s0 = 0, s1 = 0, s2 = 0, p0 = 0, p1 = 0;
        double ph_first = 0.0;
        for (const auto& s : buf_) {
            double ph = std::atan2(s.y - cy, s.x - cx);
            if (!first) {
                while (ph + offset - prev > kPi) offset -= 2.0 * kPi;
                while (ph + offset - prev < -kPi) offset += 2.0 * kPi;
            } else {
                ph_first = ph;
            }
            ph += offset;
            prev = ph;
            first = false;

            const double u = s.t - tm;
            s0 += 1.0; s1 += u; s2 += u * u;
            p0 += ph; p1 += u * ph;
        }
        const double det = s0 * s2 - s1 * s1;
        if (!(std::fabs(det) > 1e-12)) return r;
        const double slope = (s0 * p1 - s1 * p0) / det;
        const double intercept = (p0 - slope * s1) / s0;
        const double t_ref = buf_.back().t;

        // 3) 拟合质量：残差 + 最近几个点的相位创新。
        //    这两项是“换档污染”和“静止噪声团”的唯一可靠判据（见 Result 里的说明）。
        double sum_sq = 0.0;
        double prev_ph = 0.0;
        double off = 0.0;
        double tail_sum = 0.0;
        bool first_ph = true;
        const std::size_t tail_n = std::min<std::size_t>(5, buf_.size());
        const std::size_t tail_from = buf_.size() - tail_n;
        std::size_t idx = 0;
        for (const auto& s : buf_) {
            const double dx = s.x - cx;
            const double dy = s.y - cy;
            const double dist = std::sqrt(dx * dx + dy * dy);
            const double err = dist - radius;
            sum_sq += err * err;

            double ph = std::atan2(dy, dx);
            if (!first_ph) {
                while (ph + off - prev_ph > kPi) off -= 2.0 * kPi;
                while (ph + off - prev_ph < -kPi) off += 2.0 * kPi;
            }
            ph += off;
            prev_ph = ph;
            first_ph = false;

            if (idx >= tail_from) {
                const double pred = intercept + slope * (s.t - tm);
                tail_sum += std::fabs(ph - pred);
            }
            ++idx;
        }

        r.ok = true;
        r.cx = cx; r.cy = cy; r.r = radius;
        r.omega = slope;
        r.t_ref = t_ref;
        r.phase_ref = intercept + slope * (t_ref - tm);
        r.phase_span = prev - ph_first;
        r.rel_residual = std::sqrt(sum_sq / n) / std::max(radius, 1e-9);
        r.tail_innovation = tail_sum / static_cast<double>(tail_n);
        return r;
    }

private:
    struct Sample {
        double t = 0.0, x = 0.0, y = 0.0;
    };
    static constexpr double kPi = 3.14159265358979323846;

    static bool solve3(double a[3][3], double b[3], double x[3]) {
        const double det =
            a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
            a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
            a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
        if (!(std::fabs(det) > 1e-12)) return false;
        for (int k = 0; k < 3; ++k) {
            double mm[3][3];
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j) mm[i][j] = (j == k) ? b[i] : a[i][j];
            x[k] = (mm[0][0] * (mm[1][1] * mm[2][2] - mm[1][2] * mm[2][1]) -
                    mm[0][1] * (mm[1][0] * mm[2][2] - mm[1][2] * mm[2][0]) +
                    mm[0][2] * (mm[1][0] * mm[2][1] - mm[1][1] * mm[2][0])) / det;
        }
        return true;
    }

    std::deque<Sample> buf_;
    std::size_t max_samples_;
    double max_age_;

    bool locked_ = false;
    double lock_cx_ = 0.0;
    double lock_cy_ = 0.0;
    double lock_r_ = 0.0;
    double lock_omega_ = 0.0;
};

}  // namespace rune
