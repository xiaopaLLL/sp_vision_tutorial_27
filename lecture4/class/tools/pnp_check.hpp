#ifndef TOOLS__PNP_CHECK_HPP
#define TOOLS__PNP_CHECK_HPP

#include <opencv2/opencv.hpp>
#include <vector>

namespace tools
{
// 把 object_points 用解得的位姿投影回图像，返回平均重投影误差（像素）。
// 用途：自查 object_points 与 img_points 的对应顺序有没有写反。
// 顺序正确时约 2~3 px；顺序写反（例如镜像）会暴涨到几十像素以上。
inline double reprojection_error(
  const std::vector<cv::Point3f> & object_points, const std::vector<cv::Point2f> & img_points,
  const cv::Mat & rvec, const cv::Mat & tvec, const cv::Mat & camera_matrix,
  const cv::Mat & distort_coeffs)
{
  std::vector<cv::Point2f> reprojected;
  cv::projectPoints(object_points, rvec, tvec, camera_matrix, distort_coeffs, reprojected);

  double sum = 0.0;
  for (std::size_t i = 0; i < img_points.size(); i++) {
    sum += cv::norm(reprojected[i] - img_points[i]);
  }
  return sum / static_cast<double>(img_points.size());
}

}  // namespace tools

#endif  // TOOLS__PNP_CHECK_HPP
