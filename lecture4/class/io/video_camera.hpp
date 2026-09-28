#ifndef IO__VIDEO_CAMERA_HPP
#define IO__VIDEO_CAMERA_HPP

#include <chrono>
#include <opencv2/opencv.hpp>
#include <string>

#include "camera.hpp"

namespace io
{
// 从视频文件取帧，当作相机用。目的是让每个学生都能跑出同样的画面和同样的位姿数值。
class VideoCamera : public CameraBase
{
public:
  explicit VideoCamera(const std::string & config_path);

  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

  bool try_read_for(
    cv::Mat & img, std::chrono::steady_clock::time_point & timestamp,
    std::chrono::milliseconds timeout) override;

  bool is_alive() const override;

private:
  cv::VideoCapture cap_;
  bool loop_;
};

}  // namespace io

#endif  // IO__VIDEO_CAMERA_HPP
