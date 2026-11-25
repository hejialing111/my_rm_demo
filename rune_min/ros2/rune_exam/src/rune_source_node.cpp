// 输入源节点（可选）：把视频/图片/相机变成一路 sensor_msgs/Image。
//
// 考核 2.2 说的是「稳定识别**视频中**存在的扇叶」，所以要把视频喂进 ROS。
// 三种用法：
//   ros2 run rune_exam rune_source_node --ros-args -p source_type:=video -p path:=xxx.mp4
//   ros2 run rune_exam rune_source_node --ros-args -p source_type:=image -p path:=xxx.png
//   ros2 run rune_exam rune_source_node --ros-args -p source_type:=camera -p camera_index:=0
//
// 然后让识别节点改订这一路（默认 format=jpeg，发到 <topic>/compressed）：
//   ros2 run rune_exam rune_recognition_node -p compressed_topic:=/rune/image_raw/compressed
// 要发 raw 图的话两边都得改：本节点加 -p format:=raw，识别节点加
//   -p compressed_topic:='""' -p image_topic:=/rune/image_raw
//
// 视频太卡就加 -p scale:=0.5：先缩小再发，DDS 数据量和检测耗时都降到 1/4。
// 注意缩了之后像素坐标是缩小后的坐标系，算距离时内参也要跟着乘 scale。
//
// 本节点不做检测，也不跑 EKF —— 它只产图和时间戳。

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "rune_min/frame_source.hpp"

class RuneSourceNode : public rclcpp::Node {
public:
    RuneSourceNode() : Node("rune_source") {
        topic_ = declare_parameter<std::string>("topic", "/rune/image_raw");
        frame_id_ = declare_parameter<std::string>("frame_id", "camera_optical");

        rune::SourceConfig cfg;
        cfg.type = rune::source_type_from_string(
            declare_parameter<std::string>("source_type", "camera"));
        cfg.path = declare_parameter<std::string>("path", "");
        cfg.camera_index = static_cast<int>(declare_parameter<int>("camera_index", 0));
        cfg.backend = static_cast<int>(declare_parameter<int>("backend", 0));
        cfg.width = static_cast<int>(declare_parameter<int>("width", 1280));
        cfg.height = static_cast<int>(declare_parameter<int>("height", 720));
        cfg.fps = declare_parameter<double>("fps", 0.0);  // 0 = 自动
        cfg.loop = declare_parameter<bool>("loop", false);
        cfg.realtime = declare_parameter<bool>("realtime", true);
        cfg_ = cfg;

        // 发布前缩小。1440x1080 的原始图 30 fps 是 140 MB/s，虚拟机扛不住；
        // 缩到 0.5 就只剩 1/4，检测也快 4 倍。
        scale_ = declare_parameter<double>("scale", 1.0);
        if (scale_ > 0.0 && scale_ < 0.999) {
            RCLCPP_INFO(get_logger(), "发布前缩小 x%.2f", scale_);
        } else {
            scale_ = 1.0;
        }

        // 发布格式：jpeg -> <topic>/compressed（sensor_msgs/CompressedImage）
        //           raw  -> <topic>（sensor_msgs/Image）
        //
        // 1440x1080 的 raw 图 30 fps 是 140 MB/s，虚拟机必卡；JPEG 后只有 1~2 MB/s，
        // 可以全分辨率 30 帧跑。识别节点本来就能吃压缩图（compressed_topic）。
        const std::string format = declare_parameter<std::string>("format", "jpeg");
        jpeg_ = (format != "raw");
        quality_ = static_cast<int>(declare_parameter<int>("quality", 85));

        pub_ = create_publisher<sensor_msgs::msg::Image>(
            topic_, rclcpp::SensorDataQoS());
        pub_jpeg_ = create_publisher<sensor_msgs::msg::CompressedImage>(
            topic_ + "/compressed", rclcpp::SensorDataQoS());

        RCLCPP_INFO(get_logger(), "source=%s topic=%s%s format=%s fps=%.1f(0=auto)",
                    rune::source_type_to_string(cfg_.type), topic_.c_str(),
                    jpeg_ ? "/compressed" : "", jpeg_ ? "jpeg" : "raw", cfg_.fps);
    }

    bool start() {
        source_ = rune::make_frame_source(cfg_.type);
        if (!source_) {
            RCLCPP_ERROR(get_logger(), "未知的 source_type");
            return false;
        }
        if (!source_->open(cfg_)) {
            RCLCPP_ERROR(get_logger(), "打开输入源失败（%s）", source_->name());
            return false;
        }
        RCLCPP_INFO(get_logger(), "输入源就绪：%s", source_->name());

        if (cfg_.type != rune::SourceType::Camera) {
            wait_for_subscriber();
            wait_after_publish_ = true;
        }
        return true;
    }

    // 阻塞式主循环：取帧 -> 打时间戳 -> publish。
    void run() {
        rune::Frame frame;
        // 回放类源用 frame.stamp（视频是 index/fps，单图是 0）做相对时间轴，
        // 再加一个“现在”做原点，这样 header.stamp 仍然是合理的绝对时间。
        const rclcpp::Time replay_origin = now();
        std::int64_t index = 0;

        while (rclcpp::ok() && source_->next(frame)) {
            if (frame.bgr.empty()) continue;

            std_msgs::msg::Header header;
            header.frame_id = frame_id_;
            if (cfg_.type == rune::SourceType::Camera) {
                // 实时相机：用“现在”，因为它就是刚采到的。
                header.stamp = now();
            } else {
                // 图片/视频：时间轴由 IFrameSource 给，保证 dt 恒等于 1/fps，
                // 和真实机器快慢无关 —— 这样回放结果可复现。
                header.stamp =
                    replay_origin + rclcpp::Duration::from_seconds(frame.stamp);
            }
            ++index;

            if (scale_ < 0.999) {
                cv::Mat small;
                cv::resize(frame.bgr, small, cv::Size(), scale_, scale_, cv::INTER_AREA);
                frame.bgr = small;
            }

            if (jpeg_) {
                std::vector<uchar> buf;
                const std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, quality_};
                if (!cv::imencode(".jpg", frame.bgr, buf, params)) continue;
                sensor_msgs::msg::CompressedImage msg;
                msg.header = header;
                msg.format = "jpeg";
                msg.data = std::move(buf);
                pub_jpeg_->publish(msg);
            } else {
                auto msg = cv_bridge::CvImage(header, "bgr8", frame.bgr).toImageMsg();
                pub_->publish(*msg);
            }
        }

        RCLCPP_INFO(get_logger(), "输入源结束，共 %ld 帧", static_cast<long>(index));
        source_->close();

        // 单图/短视频：发完就走会把还在 DDS 缓冲里的最后一帧一起带走。
        if (wait_after_publish_) {
            rclcpp::sleep_for(std::chrono::milliseconds(300));
        }
    }

private:
    // 当前实际发布的那一路的订阅者数量（jpeg 和 raw 是不同的话题/类型）
    size_t subscriber_count() const {
        return jpeg_ ? pub_jpeg_->get_subscription_count()
                     : pub_->get_subscription_count();
    }

    std::string active_topic() const {
        return jpeg_ ? topic_ + "/compressed" : topic_;
    }

    // 单张图片只有 1 帧。如果不等订阅者完成发现就发，这一帧会直接丢掉
    // （volatile 的消息不会补给后到的订阅者）。
    void wait_for_subscriber() {
        using namespace std::chrono_literals;
        const auto deadline = std::chrono::steady_clock::now() + 5s;
        while (rclcpp::ok() && subscriber_count() == 0 &&
               std::chrono::steady_clock::now() < deadline) {
            rclcpp::sleep_for(50ms);
        }
        if (subscriber_count() == 0) {
            RCLCPP_WARN(get_logger(),
                        "等了 5 s 仍没人订阅 %s：这一帧多半会丢。"
                        "是不是识别节点没起来（或者订阅的话题不对）？",
                        active_topic().c_str());
        } else {
            RCLCPP_INFO(get_logger(), "订阅者已就绪（%zu 个）", subscriber_count());
        }
    }

    std::string topic_;
    std::string frame_id_;
    rune::SourceConfig cfg_;
    std::unique_ptr<rune::IFrameSource> source_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    bool wait_after_publish_ = false;
    double scale_ = 1.0;
    bool jpeg_ = true;
    int quality_ = 85;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_jpeg_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RuneSourceNode>();
    if (!node->start()) {
        rclcpp::shutdown();
        return 1;
    }
    node->run();  // 采集是阻塞循环，不需要 spin
    rclcpp::shutdown();
    return 0;
}
