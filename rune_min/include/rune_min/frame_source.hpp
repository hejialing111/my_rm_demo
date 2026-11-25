#pragma once

#include <memory>
#include <string>

#include <opencv2/core.hpp>

namespace rune {

// 一帧输入。
// stamp 单位是秒，语义由具体源决定：
//   - 图片/视频回放：从 0 开始、按 fps 递增的“回放时间”（可复现）
//   - 实时相机：采集时刻的单调时钟
struct Frame {
    cv::Mat bgr;
    double stamp = 0.0;
};

// 输入源类型。加一种新源 = 加一个枚举 + 一个实现类，管线代码不动。
enum class SourceType {
    ImageFile,  // 单张图片，只出一帧
    VideoFile,  // 视频文件，按 fps 回放
    Camera      // 实时相机（UVC / 工业相机）
};

// 所有“需要你按现场情况填”的采集参数都在这里。
struct SourceConfig {
    SourceType type = SourceType::Camera;

    std::string path;        // type = ImageFile/VideoFile 时必填
    int camera_index = 0;    // type = Camera 时的设备索引
    int backend = 0;         // cv::VideoCaptureAPIs；0 = 让 OpenCV 自己选
    int width = 1280;        // 期望分辨率（相机可能忽略）
    int height = 720;
    double fps = 0.0;        // <=0 = 自动：视频用文件自带帧率，相机用设备默认
    bool loop = false;       // 视频播完是否从头再来
    bool realtime = true;    // 视频是否按 fps 限速；false = 尽快跑（离线批处理）
};

// 输入源接口：只要实现这 5 个函数，就能插进整条链路。
class IFrameSource {
public:
    virtual ~IFrameSource() = default;

    virtual bool open(const SourceConfig& config) = 0;

    // 取下一帧。返回 false 表示流已结束（图片只有一帧）或读取失败。
    virtual bool next(Frame& out) = 0;

    virtual void close() = 0;

    // 日志用，比如 "video_file"。
    virtual const char* name() const = 0;

    // 这一路输入的实际帧率。录制标注视频时按它写，否则 60 fps 的源会被录成 30 fps 慢放。
    virtual double fps() const = 0;
};

// 工厂：按类型创建源。
std::unique_ptr<IFrameSource> make_frame_source(SourceType type);

// 字符串 <-> 枚举，ROS 参数里用字符串传，避免魔法数字。
SourceType source_type_from_string(const std::string& name);
const char* source_type_to_string(SourceType type);

}  // namespace rune
