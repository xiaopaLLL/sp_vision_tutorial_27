#include "camera.hpp"

#include <stdexcept>

#include "tools/yaml.hpp"
#include "video_camera.hpp"

#ifdef WITH_HIKROBOT
#include "hikrobot/hikrobot.hpp"
#endif

namespace io
{
Camera::Camera(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto source = tools::read<std::string>(yaml, "source");

  if (source == "video") {
    camera_ = std::make_unique<VideoCamera>(config_path);
    return;
  }

  if (source == "camera") {
#ifdef WITH_HIKROBOT
    camera_ = std::make_unique<HikRobot>(
      yaml["camera"]["exposure_ms"].as<double>(), yaml["camera"]["gain"].as<double>(),
      yaml["camera"]["vid_pid"].as<std::string>());
    return;
#else
    throw std::runtime_error(
      "source is 'camera' but this build has no HikRobot support. "
      "Reconfigure with -DWITH_HIKROBOT=ON after installing the MVS SDK, "
      "or set source: video in configs/yolo.yaml.");
#endif
  }

  throw std::runtime_error("Unknown source: " + source + " (expected 'video' or 'camera')");
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
}

bool Camera::try_read_for(
  cv::Mat & img, std::chrono::steady_clock::time_point & timestamp,
  std::chrono::milliseconds timeout)
{
  return camera_->try_read_for(img, timestamp, timeout);
}

bool Camera::is_alive() const { return camera_->is_alive(); }

}  // namespace io
