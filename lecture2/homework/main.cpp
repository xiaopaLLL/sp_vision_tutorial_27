#include <string>

#include "io/camera.hpp"
#include "opencv2/opencv.hpp"
#include "tasks/yolo.hpp"
#include "tools/img_tools.hpp"

int main()
{
  // 初始化相机、yolo类
  Camera camera;
  auto_aim::YOLO yolo("./configs/yolo.yaml", true);  //这个实例变量到底应该命名成什么。。

  while (1) {
    // 调用相机读取图像
    cv::Mat img = camera.read();
    // cv::Mat img = cv::imread("../../lecture1/img/1.jpg");
    //自己写的时候因为没图没设备偷偷用的第一讲的图片，好像还真能跑出来（虽然有几张结果比较离谱XD

    // 调用yolo识别装甲板
    std::list<auto_aim::Armor> armors = yolo.detect(img);
    /*
    这里真的是难为死我了，觉得是翻源码最久的时候
    首先yolo这个实例它配置了自己的yaml，所以用这个实例的detect方法就可以实现检测
    然后检测完返回的是一个元素类型为Armor的链表，Armor里有一个std::vector<cv::Point2f>可以传入tool::draw_points实现画图
    至于为什么能返回这个链表我研究了半天
    右键yolo.hpp里那个YOLOBase的detect点转到定义会跳到apriltag_detector.cpp的地方，单这个就误导了我非常久（后面才知道这个tm是后一题的。。
    YOLO这个类，提供了detect方法，作用是让它的private指针变量yolo_指向的YOLOBase执行detect方法
    YOLOBase里的detect就非常抽象，只在头文件里定义了一行virtual
    以我薄弱的c++功底根本不知道这是什么玩意，不过好在经过非常多时间的查阅和询问ai我终于找到了所谓的detect
    这个YOLO构建实例的时候把yolo_设成了一个叫YOLOV5的类
    这个YOLOV5(tasks/yolov5.cpp)中终于找到了能返回Armor链表的detect方法..虽然具体的还是不清楚hh
    */

    //遍历上面的链表画图  使用绿色  thickness=2
    for (auto armor : armors) {
      tools::draw_points(img, armor.points, cv::Scalar(0, 255, 0), 2);
      // 因为看到函数里有draw_text和Armor结构体里也存了颜色和类型
      // 作业示例图输出了bluefour 所以我也写了一下
      // 不过用的是points的左上角的坐标，好像和示例不太一样，我好像不知道这是怎么做到的..
      std::string szText = "";
      szText += auto_aim::COLORS[(int)armor.color];
      szText += auto_aim::ARMOR_NAMES[(int)armor.name];
      tools::draw_text(img, szText, armor.points[0], cv::Scalar(0, 255, 0), 1.2, 2);
    }

    // 显示图像
    cv::resize(img, img, cv::Size(640, 480));
    cv::imshow("img", img);
    if (cv::waitKey(1) == 'q') {
      // 不是学长也太坏了这里写个0，说是要改成1，要不是问了一下说要连续采样都不敢动这里
      break;
    }
  }

  return 0;
}