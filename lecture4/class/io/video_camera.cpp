#include "video_camera.hpp"

#include <stdexcept>

#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace io
{
VideoCamera::VideoCamera(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto path = tools::read<std::string>(yaml["video"], "path");
  loop_ = yaml["video"]["loop"] ? yaml["video"]["loop"].as<bool>() : true;

  cap_.open(path);
  if (!cap_.isOpened()) {
    tools::logger()->error("[VideoCamera] Failed to open video: {}", path);
    throw std::runtime_error("Failed to open video: " + path);
  }

  tools::logger()->info(
    "[VideoCamera] {} ({}x{}, {} frames, loop={})", path,
    static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH)),
    static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT)),
    static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_COUNT)), loop_);
}

void VideoCamera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  cap_ >> img;
  if (img.empty() && loop_) {
    cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
    cap_ >> img;
  }
  timestamp = std::chrono::steady_clock::now();
}

bool VideoCamera::try_read_for(
  cv::Mat & img, std::chrono::steady_clock::time_point & timestamp, std::chrono::milliseconds)
{
  // 视频没有"等一帧"的语义，直接读，用返回值表达成功与否
  read(img, timestamp);
  return !img.empty();
}

bool VideoCamera::is_alive() const { return cap_.isOpened(); }

}  // namespace io
