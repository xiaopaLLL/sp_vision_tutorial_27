#include <iostream>
#include <opencv2/opencv.hpp>

cv::Mat addNoiseColor(const cv::Mat & src)
{
  CV_Assert(src.type() == CV_8UC3);  // 输入必须是 8-bit unsigned, 3 channels

  cv::Mat dst = src.clone();

  // 创建随机噪声
  cv::Mat noise(src.rows, src.cols, CV_8UC3);
  cv::randu(noise, 0, 50);

  // 给图像加入噪声
  cv::add(dst, noise, dst);

  return dst;
}
int main()
{
  cv::Mat img = cv::imread("image.jpg", cv::IMREAD_COLOR);
  if (img.empty()) {
    std::cerr << "image.jpg not found!" << std::endl;
    return -1;
  }

  cv::Mat noisy = addNoiseColor(img);

  // 缩放原始图像和噪声图像
  cv::Mat img_resized, noisy_resized;
  cv::resize(img, img_resized, cv::Size(), 0.5, 0.5);
  cv::resize(noisy, noisy_resized, cv::Size(), 0.5, 0.5);

  cv::imshow("original", img_resized);  // 显示 original 图像
  cv::imshow("noisy", noisy_resized);   // 显示 noisy 图像
  cv::waitKey(0);                       // 暂停程序，按任意键退出
  return 0;
}