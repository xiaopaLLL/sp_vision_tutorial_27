#include <chrono>
#include <opencv2/opencv.hpp>

#include "fmt/core.h"
#include "io/camera.hpp"
#include "tasks/yolo.hpp"
#include "tools/img_tools.hpp"

// clang-format off
//  相机内参
static const cv::Mat camera_matrix =
    (cv::Mat_<double>(3, 3) <<  1286.307063384126 , 0                  , 645.34450819155256,
                                0                 , 1288.1400736562441 , 483.6163720308021 ,
                                0                 , 0                  , 1                   );
// 畸变系数
static const cv::Mat distort_coeffs =
    (cv::Mat_<double>(1, 5) << -0.47562935060124745, 0.21831745829617311, 0.0004957613589406044, -0.00034617769548693592, 0);
// clang-format on

static const double LIGHTBAR_LENGTH = 0.056; // 灯条长度    单位：米
static const double ARMOR_WIDTH = 0.135;     // 装甲板宽度  单位：米

// #### Task 01 ############################################
// object_points 是 物体局部坐标系下 n个点 的坐标。
// 对于我们而言，也就是装甲板坐标系下4个点的坐标。
// 请你填写下面的 object_points:
//
static const std::vector<cv::Point3f> object_points {
    { -ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2, 0 },  // 点 1
    { ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2, 0 },   // 点 2
    { ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2, 0 },    // 点 3
    { -ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2, 0 }    // 点 4
};
//
// 提示：
// - 装甲板坐标系是三维的坐标系，但是四个点都在 z 坐标为 0 的平面上，所以已经为你填写了四个 0 。
// - 在上方定义有 灯条长度 和 装甲板宽度，你应当用 "± ARMOR_WIDTH / 2" 这样的写法来填写。
// - 点序规定：左上、右上、右下、左下（自左上顺时针）。这个顺序必须和 Task02 的 img_points 一一对应。
//   ⚠ Armor::points 的真实顺序就是上面这个，不是 tasks/armor.hpp 注释里写的那个。
// #########################################################

int main(int argc, char *argv[])
{
    auto_aim::YOLO detector("configs/yolo.yaml");
    io::Camera camera("configs/yolo.yaml");

    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;

    while (true)
    {
        camera.read(img, timestamp);   // 读视频还是读相机，由 configs/yolo.yaml 的 source 决定
        if (img.empty())               // 读取失败 或 视频结尾（loop: true 时不会发生）
            break;

        auto armors = detector.detect(img);

        if (!armors.empty())
        {
            auto armor = armors.front();           // 取第一个装甲板
            tools::draw_points(img, armor.points); // 绘制装甲板 4 个关键点
            // // 临时：验证 armor.points 的真实顺序
            // for (int i = 0; i < 4; i++)
            //     tools::draw_text(img, std::to_string(i), armor.points[i], cv::Scalar(255, 0, 255), 1.5, 3);
            // fmt::print("0:({:.0f},{:.0f}) 1:({:.0f},{:.0f}) 2:({:.0f},{:.0f}) 3:({:.0f},{:.0f})\n",
            //         armor.points[0].x, armor.points[0].y, armor.points[1].x, armor.points[1].y,
            //         armor.points[2].x, armor.points[2].y, armor.points[3].x, armor.points[3].y);
                
            // #### Task 02 ############################################
            // img_points 是 像素坐标系下 n个点 的坐标，也就是照片上装甲板 4 个点的坐标。
            // 请你填写下面的 img_points:
            //
            std::vector<cv::Point2f> img_points{armor.points[0], armor.points[1], armor.points[2], armor.points[3]};
            //
            // 提示：
            // - armor.points 就是 YOLO 给出的那 4 个关键点。
            // - 顺序必须与 Task01 的 object_points 一一对应：左上、右上、右下、左下。
            // #########################################################



            // #### Task 03 ############################################
            cv::Mat rvec, tvec;
            // 所有要传入的值都已经具备了。现在调用 solvePnP 解算装甲板位姿，
            // rvec 和 tvec 用于存储 solvePnP 输出的结果。
            // 你需要在下面填写 输入给 solvePnP 的参数：
            //
            cv::solvePnP(object_points, img_points, camera_matrix, distort_coeffs, rvec, tvec);
            //
            // #########################################################



            // #### Task 04 ############################################
            // 现在，draw_text 只打印 0.0
            // 请你改写下面draw_text的参数，把解得的 tvec 和 rvec 打印出来
            //
            tools::draw_text(img, fmt::format("tvec:  x{: .2f} y{: .2f} z{: .2f}", tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)), cv::Point(10, 60), cv::Scalar(0, 255, 255), 1.7, 3);
            tools::draw_text(img, fmt::format("rvec:  x{: .2f} y{: .2f} z{: .2f}", rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2)), cv::Point(10, 120), cv::Scalar(0, 255, 255), 1.7, 3);
            //
            // 提示：
            // - 使用 tvec.at<double>(0)，可以得到一个double变量，它是tvec中首个元素的值。
            // #########################################################



            // #### Task 05 ############################################
            // 使用 cv::Rodrigues ，把 rvec 旋转向量转换为 rmat 旋转矩阵。
            // 再使用反三角函数，把旋转矩阵 rmat 中的元素转化为欧拉角，并在画面上显示。
            //
            cv::Mat rmat;
            cv::Rodrigues(rvec, rmat);
            double yaw = atan2(rmat.at<double>(0, 2), rmat.at<double>(2, 2));
            double pitch = asin(rmat.at<double>(1, 2));
            double roll = atan2(rmat.at<double>(1, 0), rmat.at<double>(1, 1));
            // 这里可以使用一些数学公式来计算欧拉角，但为了简化，我们暂时用0填充
            tools::draw_text(img, fmt::format("euler angles:  yaw{: .2f} pitch{: .2f} roll{: .2f}", yaw, pitch, roll), cv::Point(10, 180), cv::Scalar(0, 255, 255), 1.7, 3);
            //
            // 提示：
            // - cv::Mat 的下标从0开始，而不是1。
            // - 从cv::Mat 中取元素的方法和上面的 tvec 类似。如： rmat.at<double>(0, 2)
            // #########################################################
        }

        cv::imshow("press q to quit, space to pause", img);

        int key = cv::waitKey(20);
        if (key == 'q')
            break;
        if (key == ' ')                 // 空格暂停，再按任意键继续
            cv::waitKey(0);
    }

    cv::destroyAllWindows();
    return 0;
}
