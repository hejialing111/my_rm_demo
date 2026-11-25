#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/header.hpp>

#include "rune_exam/msg/blade_keypoints.hpp"
#include "rune_min/blade_detector.hpp"
#include "rune_min/config.hpp"
#include "rune_min/preprocess.hpp"

//解码，检测，换算坐标系并发布

class RuneRecognitionNode : public rclcpp::Node {
public:
//构造函数，初始化节点，声明参数，设置订阅和发布器
    RuneRecognitionNode() : Node("rune_recognition") {
        compressed_topic_ = declare_parameter<std::string>(
            "compressed_topic", "/rm_energy/camera/image_raw/compressed");
        keypoints_topic_ = declare_parameter<std::string>(
           "keypoints_topic", "/rune/blade_keypoints");

    //这里声明了几个调试相关的参数，分别是是否发布调试图像、调试图像的帧率和缩放比例
        debug_image_ = declare_parameter<bool>("publish_debug_image", false);
        debug_fps_ = declare_parameter<double>("debug_fps", 15.0);
        debug_scale_ = declare_parameter<double>("debug_scale", 1.0);
        if (debug_scale_ <= 0.0 || debug_scale_ > 1.0) debug_scale_ = 1.0;
        if (debug_image_) {
            //发布器，用于发布调试图像
            pub_debug_ = create_publisher<sensor_msgs::msg::CompressedImage>(
                "/rune/debug/image/compressed", rclcpp::SensorDataQoS());
        }
//叶片检测
        rune::BladeDetectConfig cfg;

        const std::string color =
            declare_parameter<std::string>("rune_color", "both");
        if (color == "red") {
            cfg.color = rune::RuneColor::Red;
        } else if (color == "blue") {
            cfg.color = rune::RuneColor::Blue;
        } else {
            cfg.color = rune::RuneColor::Both;
        }
//处理两种颜色HSV阈值
        const double s_min = declare_parameter<double>("red_s_min", cfg.red1_low[1]);
        const double v_min = declare_parameter<double>("red_v_min", cfg.red1_low[2]);
        const double h1_max = declare_parameter<double>("red_h1_max", cfg.red1_high[0]);
        const double h2_min = declare_parameter<double>("red_h2_min", cfg.red2_low[0]);
        cfg.red1_low = cv::Scalar(0.0, s_min, v_min);
        cfg.red1_high = cv::Scalar(h1_max, 255.0, 255.0);
        cfg.red2_low = cv::Scalar(h2_min, s_min, v_min);
        cfg.red2_high = cv::Scalar(180.0, 255.0, 255.0);
//最小和最大面积比率，形态学操作的核大小，最小纵横比，最小椭圆填充率，最小固体度，最小半径像素
        cfg.min_area_ratio = declare_parameter<double>("min_area_ratio", cfg.min_area_ratio);
        cfg.max_area_ratio = declare_parameter<double>("max_area_ratio", cfg.max_area_ratio);
        cfg.morph_px = static_cast<int>(declare_parameter<int>("morph_px", cfg.morph_px));
        cfg.min_aspect = declare_parameter<double>("min_aspect", cfg.min_aspect);
        cfg.min_ellipse_fill =
            declare_parameter<double>("min_ellipse_fill", cfg.min_ellipse_fill);
        cfg.min_solidity = declare_parameter<double>("min_solidity", cfg.min_solidity);
        cfg.min_radius_px = declare_parameter<double>("min_radius_px", cfg.min_radius_px);

        // 剔除机关中心的 R 标：半径 scale×r 的圆周上，亮点占比超过 max 就判为 R 标。
        cfg.hub_ring_scale = declare_parameter<double>("hub_ring_scale", cfg.hub_ring_scale);
        cfg.hub_ring_coverage_max =
            declare_parameter<double>("hub_ring_coverage_max", cfg.hub_ring_coverage_max);
        //比本帧最大的小非常多的候选就丢
        cfg.min_radius_ratio_to_max = declare_parameter<double>(
            "min_radius_ratio_to_max", cfg.min_radius_ratio_to_max);
        cfg.max_blades = static_cast<int>(declare_parameter<int>("max_blades", cfg.max_blades));
        //避免"形状不达标的亮斑"也被当成靶发出去。
        cfg.allow_shape_fallback =
            declare_parameter<bool>("allow_shape_fallback", cfg.allow_shape_fallback);
//这里创建了一个 BladeDetector 对象，用于检测亮靶
        detector_ = std::make_unique<rune::BladeDetector>(cfg);

        //去畸变 / ROI / 缩放
        // const std::string calib_file = declare_parameter<std::string>("calib_file", "");
        const bool enable_undistort = declare_parameter<bool>("enable_undistort", true);
        // const int roi_w = static_cast<int>(declare_parameter<int>("roi_width", 0));
        // const int roi_h = static_cast<int>(declare_parameter<int>("roi_height", 0));
        const double detect_scale = declare_parameter<double>("detect_scale", 1.0);
        const double rc_x = declare_parameter<double>("r_center_x", -1.0);
        const double rc_y = declare_parameter<double>("r_center_y", -1.0);

        // 创建 RuneConfig 对象，并根据参数设置相机标定、去畸变、ROI 和缩放等配置
        rune::RuneConfig core_cfg;
        core_cfg.enable_undistort = enable_undistort;
        core_cfg.detect_scale = detect_scale;
        // core_cfg.roi_size = cv::Size(roi_w, roi_h);
        core_cfg.r_center_px =
            cv::Point2f(static_cast<float>(rc_x), static_cast<float>(rc_y));
            //加载相机标定参数
        // if (!calib_file.empty()) {
        //     if (rune::load_camera_calib(calib_file, core_cfg.K, core_cfg.D)) {
        //         RCLCPP_INFO(get_logger(), "已加载相机标定: %s", calib_file.c_str());
        //     } else {
        //         RCLCPP_WARN(get_logger(),
        //                     "读不到标定文件，本次用原始图像: %s", calib_file.c_str());
        //     }
        // }
//这里根据是否需要去畸变和 ROI/缩放来创建 FramePreprocessor 对象，并设置相关参数
        const bool want_geo =
            (std::abs(detect_scale - 1.0)) > 1e-9;
        pre_ = std::make_unique<rune::FramePreprocessor>(core_cfg);
        if (pre_->undistorted() && want_geo) {
            RCLCPP_WARN(get_logger(),
                        "去畸变与 ROI/缩放不能同时用（坐标无法无损换算回原图），"
                        "本次忽略 ROI/缩放");
            core_cfg.roi_size = cv::Size(0, 0);
            core_cfg.detect_scale = 1.0;
            pre_ = std::make_unique<rune::FramePreprocessor>(core_cfg);
        }
//创建发布器和订阅器，订阅压缩图像消息，并绑定回调函数
        pub_keypoints_ = create_publisher<rune_exam::msg::BladeKeypoints>(
            keypoints_topic_, 10);
        sub_compressed_ = create_subscription<sensor_msgs::msg::CompressedImage>(
            compressed_topic_, rclcpp::SensorDataQoS(),
            std::bind(&RuneRecognitionNode::on_compressed, this,
                      std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "recognition: %s -> %s",
                    compressed_topic_.c_str(), keypoints_topic_.c_str());
    }

private:
//这里on_compressed函数是订阅压缩图像消息的回调函数，它将压缩图像解码为BGR格式，并调用handle函数进行处理
    void on_compressed(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        if (msg->data.empty()) return;
        const cv::Mat buffer(1, static_cast<int>(msg->data.size()), CV_8UC1,
                             const_cast<uint8_t*>(msg->data.data()));
        const cv::Mat bgr = cv::imdecode(buffer, cv::IMREAD_COLOR);
        handle(bgr, msg->header);
    }

    void handle(const cv::Mat& bgr, const std_msgs::msg::Header& header) {
        if (bgr.empty()) return;
        const auto t_begin = std::chrono::steady_clock::now();

        // apply()函数返回原图前拷贝拷贝
        const cv::Mat& view = pre_->apply(bgr);
        if (view.empty()) return;
//这里调用 BladeDetector 的 detect 函数进行亮靶检测，并记录处理时间
        rune::RuneGeometry geom;
        std::vector<rune::Blade> blades = detector_->detect(view, &geom);
        const double process_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_begin).count();

        // 检测是在 ROI / 缩放后的 view 上做的，这里换算回原图坐标
        // 这样发布出去的 keypoints 和 CameraInfo 永远是同一套坐标系。
        const cv::Rect roi = pre_->roi();
        const double s = pre_->scale();
        const bool identity = std::abs(s - 1.0) < 1e-9 &&
                              roi.width == bgr.cols && roi.height == bgr.rows;
                              //如果没有缩放且 ROI 覆盖了整幅图，则不需要换算坐标
        if (!identity) {
            //inv表示缩放的倒数，ox和oy表示ROI的左上角坐标
            const float inv = static_cast<float>(1.0 / s);
            const float ox = static_cast<float>(roi.x);
            const float oy = static_cast<float>(roi.y);
            for (auto& b : blades) {
                //对于每个检测到的叶片，换算其中心点和关键点的坐标，并调整半径、半长轴和面积
                b.center = cv::Point2f(b.center.x * inv + ox, b.center.y * inv + oy);
                for (auto& kp : b.keypoints) {
                    kp = cv::Point2f(kp.x * inv + ox, kp.y * inv + oy);
                }
                b.radius_px *= inv;
                b.semi_minor_px *= inv;
                b.area_px /= (s * s);
            }

            //如果检测到机关圆心和相位有效，则换算其中心点坐标和半径
            if (geom.valid) {
                geom.center = cv::Point2f(geom.center.x * inv + ox,
                                          geom.center.y * inv + oy);
                geom.radius_px *= inv;
            }
        }




//创建并发布 BladeKeypoints 消息，其中包含检测到的叶片的中心点、半径、关键点坐标，以及机关圆心和相位信息
        rune_exam::msg::BladeKeypoints out;
        //header是ROS消息的标准头部，包含时间戳和坐标系信息
        out.header = header;
        out.blades.reserve(blades.size());
        for (const auto& blade : blades) {
            rune_exam::msg::Blade msg;
            msg.id = blade.id;
            msg.confidence = blade.confidence;
            msg.center.x = blade.center.x;
            msg.center.y = blade.center.y;
            msg.center.z = 0.0;
            msg.radius_px = blade.radius_px;
            msg.semi_minor_px = blade.semi_minor_px;
            msg.angle_rad = blade.angle_rad;
            msg.area_px = static_cast<float>(blade.area_px);
            msg.hole_count = blade.hole_count;
            msg.keypoints.reserve(blade.keypoints.size());
            //对于每个关键点，将其坐标转换为geometry_msgs::msg::Point类型，并添加到消息中，这里的geometry_msgs::msg::Point是ROS中表示三维点的消息类型，包含x、y、z三个坐标分量
            for (const auto& kp : blade.keypoints) {
                geometry_msgs::msg::Point p;
                p.x = kp.x;
                p.y = kp.y;
                p.z = 0.0;
                msg.keypoints.push_back(p);
            }
            //将每个叶片消息添加到 BladeKeypoints 消息的 blades 向量中
            out.blades.push_back(std::move(msg));
        }
        //将机关圆心和相位信息添加到 BladeKeypoints 消息中，并发布该消息
        out.rune_valid = geom.valid;
        out.rune_center.x = geom.center.x;
        out.rune_center.y = geom.center.y;
        out.rune_center.z = 0.0;
        out.rune_radius_px = geom.radius_px;
        pub_keypoints_->publish(out);
//如果启用了调试图像发布，则调用publish_debug函数发布调试图像
        if (debug_image_) publish_debug(bgr, view, header, blades);
//打印处理时间，ROI大小，检测到叶片数量，机关圆心等信息，每两秒一次
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "处理 %.1f ms（预处理 %.1f ms，ROI %dx%d），blades=%zu "
                             "center=(%.0f,%.0f) r=%.0f phase=%.2f",
                             process_ms, pre_->last_cost_ms(),
                             roi.width, roi.height, blades.size(),
                             geom.valid ? geom.center.x : -1.0f,
                             geom.valid ? geom.center.y : -1.0f,
                             geom.radius_px, geom.phase_rad);
    }

    // 调试图限频/缩图后发 raw Image
    void publish_debug(const cv::Mat& bgr, const cv::Mat& view,
                       const std_msgs::msg::Header& header,
                       const std::vector<rune::Blade>& blades) {
        const double t = this->now().seconds();
        //如果调试帧率大于0且距离上次调试时间小于调试间隔，则直接返回，不发布调试图像
        if (debug_fps_ > 0.0 && (t - last_debug_s_) < 1.0 / debug_fps_) return;
        last_debug_s_ = t;

        // 关键点已换算回原图坐标，所以画在原图上；开了去畸变时画在 view 上。
        const cv::Mat& canvas = pre_->undistorted() ? view : bgr;
        cv::Mat dbg = canvas.clone();
        draw(dbg, blades);
        if (debug_scale_ < 0.999) {
            cv::Mat small;
            cv::resize(dbg, small, cv::Size(), debug_scale_, debug_scale_,
                       cv::INTER_AREA);
            dbg = small;
        }
//这里将调试图像编码为JPEG格式，并发布为sensor_msgs::msg::CompressedImage消息
        std::vector<uchar> buf;
        const std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, 85};
        if (!cv::imencode(".jpg", dbg, buf, params)) return;
        sensor_msgs::msg::CompressedImage msg;
        msg.header = header;
        msg.format = "jpeg";
        msg.data = std::move(buf);
        pub_debug_->publish(msg);
    }
//这里的draw函数用于在图像上绘制检测到的叶片信息，包括叶片的圆形轮廓、中心点、关键点和ID等
    static void draw(cv::Mat& img, const std::vector<rune::Blade>& blades) {
        for (const auto& b : blades) {
            const cv::Point c(static_cast<int>(b.center.x),
                              static_cast<int>(b.center.y));
            cv::circle(img, c, static_cast<int>(b.radius_px),
                       cv::Scalar(0, 255, 0), 2);
            cv::drawMarker(img, c, cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 24, 2);

            for (size_t i = 0; i < b.keypoints.size(); ++i) {
                const cv::Point kp(static_cast<int>(b.keypoints[i].x),
                                   static_cast<int>(b.keypoints[i].y));
                cv::circle(img, kp, 5, cv::Scalar(255, 0, 255), -1);
                cv::putText(img, std::to_string(i), kp + cv::Point(6, -6),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7,
                            cv::Scalar(255, 0, 255), 2);
            }

            cv::putText(img, "id=" + std::to_string(b.id), c + cv::Point(-20, -40),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
        }
    }

    std::string compressed_topic_;
    std::string keypoints_topic_;

    bool debug_image_ = false;
    double debug_fps_ = 15.0;
    double debug_scale_ = 0.5;
    double last_debug_s_ = -1e9;

    std::unique_ptr<rune::BladeDetector> detector_;
    std::unique_ptr<rune::FramePreprocessor> pre_;
//这里定义了订阅器和发布器，用于接收压缩图像消息和发布叶片关键点消息以及调试图像消息
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_compressed_;
    rclcpp::Publisher<rune_exam::msg::BladeKeypoints>::SharedPtr pub_keypoints_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_debug_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RuneRecognitionNode>());
    rclcpp::shutdown();
    return 0;
}
