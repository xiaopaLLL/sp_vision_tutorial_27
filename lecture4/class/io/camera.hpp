#ifndef IO__CAMERA_HPP
#define IO__CAMERA_HPP

#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

namespace io
{
// 取帧源的抽象接口。队内的 HikRobot 与教学用的 VideoCamera 都实现它。
class CameraBase
{
public:
  virtual ~CameraBase() = default;

  virtual void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) = 0;

  virtual bool try_read_for(
    cv::Mat & img, std::chrono::steady_clock::time_point & timestamp,
    std::chrono::milliseconds timeout) = 0;

  virtual bool is_alive() const = 0;
};

// 门面：按 configs 里的 source 决定用哪个实现，调用方不用关心
class Camera
{
public:
  explicit Camera(const std::string & config_path);

  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);

  bool try_read_for(
    cv::Mat & img, std::chrono::steady_clock::time_point & timestamp,
    std::chrono::milliseconds timeout);

  bool is_alive() const;

private:
  std::unique_ptr<CameraBase> camera_;
};

}  // namespace io

#endif  // IO__CAMERA_HPP
