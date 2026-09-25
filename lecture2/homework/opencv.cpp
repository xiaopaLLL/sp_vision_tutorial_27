/*
有了第二问的基础上开始感觉第三问感觉挺类似的，无非是换了一些类和函数的名字
在作业pdf里截了个图(是test.png)，第一遍直接用给的yolo.yaml，结果只能识别id=18一个图
这我就犯难了，询问了一下负责人说课上讲过要自己调一些参，这才明白这问具体是要干什么，原来是自己进行调参的这个过程
没接触过什么调参手段，在这方面询问了一下deepseek，deepseek随便解释了一下，给了一大堆调参手段
复制了一份配置文件（因为不敢动原来的），把修改后的版本放在configs/yolo2.yaml了
经过尝试，只需要把adaptive_thresh_constant这个参数从7改成12(阈值好像在10~11中间)就能识别出3个图了（好像并没有deepseek说的那么复杂
这里忏悔一下，因为是真不会TT

这里是线下调试完的我，弄完才发现之前自己的神秘尝试有多糖
参数需要大动没自己试的那么轻松
首先被相机硬控了一下午之后发现不是代码的问题XD
然后学长们换了一个相机之后成功进入作业主线（其实是作业的一部分
经过挺久的神秘调参，yolo2.yaml已经被改的特别神秘了我也不知道这是怎么改出来的（反正效果好像还可以
玩的挺开心的，学长们人都很好<3
*/

#include "opencv2/opencv.hpp"

#include <string>

#include "io/camera.hpp"
#include "tasks/apriltag_detector.hpp"
#include "tasks/yolo.hpp"
#include "tools/img_tools.hpp"

int main()
{
  // 初始化相机、yolo类
  Camera camera;
  auto_charge::AprilTagDetector yolo("./configs/yolo2.yaml");
  while (1) {
    // 调用相机读取图像
    cv::Mat img = camera.read();
    // cv::Mat img = cv::imread("test.png");

    // 调用yolo识别opencv标志
    std::vector<auto_charge::TagDetection> tags = yolo.detect(img);
    //我觉得有点奇怪的一点就是说Armor那里还是链表这里就变成数组了，不是很理解这里的区别

    //遍历上面的数组画图  使用绿色  thickness=2
    for (auto tag : tags) {
      tools::draw_points(img, tag.corners, cv::Scalar(0, 255, 0), 2);
      std::string szText = "";
      szText += std::to_string(tag.id);
      //TagDetection里有一个int类型的id成员，应该代表的是检测到的id，这里用to_string的方法把他变成string显示
      tools::draw_text(img, szText, tag.center, cv::Scalar(0, 255, 0), 1.2, 2);
    }

    // 显示图像
    cv::resize(img, img, cv::Size(1280, 960));
    cv::imshow("img", img);
    if (cv::waitKey(1) == 'q') {
      break;
    }
  }

  return 0;
}