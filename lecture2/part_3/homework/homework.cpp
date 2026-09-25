#include <iostream>
#include <opencv2/opencv.hpp>

// ======================= 作业 =======================
// 1. 读取 ../assets/demo.jpg
// 2. 使用 cvtColor 把图像转为灰度图（颜色空间：BGR2GRAY）
// 3. 使用 imwrite 把灰度图保存为 gray.jpg
// 4. 在灰度图上用 circle 画一个圆，标记你要"瞄准"的位置
// 5. 显示灰度图，按任意键退出
// ====================================================

int main()
{
  // TODO: 在这里完成你的代码

  //step 1
  cv::Mat img = cv::imread("assets/demo.jpg");
  if (img.empty()) {
    std::cout << "读取图片失败！请检查路径" << std::endl;
    return -1;
  }

  //step 2
  cv::Mat gray;
  cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

  //step 3
  cv::imwrite("gray.jpg", gray);

  //step 4
  cv::circle(gray, cv::Point(400, 120), 80, cv::Scalar(0, 0, 255), 3);

  //step 5
  cv::imshow("Hello OpenCV", gray);
  std::cout << "按任意键关闭窗口..." << std::endl;
  cv::waitKey(0);

  return 0;
}
