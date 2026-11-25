#include "rune_min/frame_source.hpp"

#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
//处理输入源：读取文件，通过config里的数据给设置参数，读取下一帧，关闭
namespace rune {
namespace {

using SteadyClock = std::chrono::steady_clock;

double steady_now_s() {
    return std::chrono::duration<double>(SteadyClock::now().time_since_epoch()).count();
}

void sleep_until_s(double t) {
    const auto target = SteadyClock::time_point(
        std::chrono::duration_cast<SteadyClock::duration>(
            std::chrono::duration<double>(t)));
    std::this_thread::sleep_until(target);
}

// ---------------------------------------------------------------- 单张图片
class ImageFileSource final : public IFrameSource {
public:
    bool open(const SourceConfig& config) override {
        image_ = cv::imread(config.path);
        if (image_.empty()) {
            std::cerr << "[source] 读不到图片: " << config.path << "\n";
            return false;
        }
        served_ = false;
        return true;
    }

    bool next(Frame& out) override {
        if (served_ || image_.empty()) return false;
        out.bgr = image_.clone();
        out.stamp = 0.0;  // 单图没有时间轴
        served_ = true;
        return true;
    }

    void close() override { image_.release(); }

    const char* name() const override { return "image_file"; }

    double fps() const override { return 1.0; }  // 单图没有帧率

private:
    cv::Mat image_;
    bool served_ = true;
};

// ---------------------------------------------------------------- 视频文件
class VideoFileSource final : public IFrameSource {
public:
    bool open(const SourceConfig& config) override {
        config_ = config;
        if (!cap_.open(config.path)) {
            std::cerr << "[source] 打不开视频: " << config.path << "\n";
            return false;
        }
        if (config_.fps <= 0.0) {
            const double file_fps = cap_.get(cv::CAP_PROP_FPS);
            config_.fps = (file_fps > 1.0) ? file_fps : 30.0;
        }
        start_s_ = steady_now_s();
        index_ = 0;
        return true;
    }

    bool next(Frame& out) override {
        if (!cap_.isOpened()) return false;

        if (!cap_.read(out.bgr) || out.bgr.empty()) {
            if (!config_.loop) return false;
            cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
            index_ = 0;
            start_s_ = steady_now_s();
            if (!cap_.read(out.bgr) || out.bgr.empty()) return false;
        }

        // 回放时间轴：与真实耗时无关，保证同一段视频每次跑出的 dt 都一样。
        out.stamp = static_cast<double>(index_) / config_.fps;
        ++index_;

        if (config_.realtime) {
            sleep_until_s(start_s_ + static_cast<double>(index_) / config_.fps);
        }
        return true;
    }

    void close() override { cap_.release(); }

    const char* name() const override { return "video_file"; }

    double fps() const override { return config_.fps > 0.0 ? config_.fps : 30.0; }

private:
    SourceConfig config_;
    cv::VideoCapture cap_;
    double start_s_ = 0.0;
    std::int64_t index_ = 0;
};

// ---------------------------------------------------------------- 实时相机
// TODO(相机): 用厂商 SDK 时，只替换本类的 open()/next() 内部，
//             对外仍然是 IFrameSource，管线代码不需要任何改动。
class CameraSource final : public IFrameSource {
public:
    bool open(const SourceConfig& config) override {
        config_ = config;
        const int api = (config.backend > 0) ? config.backend : cv::CAP_ANY;
        if (!cap_.open(config.camera_index, api)) {
            std::cerr << "[source] 打不开相机 index=" << config.camera_index
                      << " backend=" << config.backend << "\n";
            return false;
        }
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, config.width);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, config.height);
        if (config.fps > 0.0) cap_.set(cv::CAP_PROP_FPS, config.fps);
        return true;
    }

    bool next(Frame& out) override {
        const double before = steady_now_s();
        if (!cap_.read(out.bgr) || out.bgr.empty()) return false;
        // 用读取完成的时刻，宁可晚一点，也不要假装是未来的帧。
        out.stamp = 0.5 * (before + steady_now_s());
        return true;
    }

    void close() override { cap_.release(); }

    const char* name() const override { return "camera"; }

    double fps() const override {
        const double dev_fps = cap_.get(cv::CAP_PROP_FPS);
        if (config_.fps > 0.0) return config_.fps;
        return dev_fps > 1.0 ? dev_fps : 30.0;
    }

private:
    SourceConfig config_;
    cv::VideoCapture cap_;
};

}  // namespace

std::unique_ptr<IFrameSource> make_frame_source(SourceType type) {
    switch (type) {
        case SourceType::ImageFile: return std::make_unique<ImageFileSource>();
        case SourceType::VideoFile: return std::make_unique<VideoFileSource>();
        case SourceType::Camera: return std::make_unique<CameraSource>();
    }
    return nullptr;
}

SourceType source_type_from_string(const std::string& name) {
    if (name == "image" || name == "image_file") return SourceType::ImageFile;
    if (name == "video" || name == "video_file") return SourceType::VideoFile;
    return SourceType::Camera;
}

const char* source_type_to_string(SourceType type) {
    switch (type) {
        case SourceType::ImageFile: return "image";
        case SourceType::VideoFile: return "video";
        case SourceType::Camera: return "camera";
    }
    return "camera";
}

}  // namespace rune
