#include <chrono>
#include <cmath>
#include <opencv2/opencv.hpp>

#include "fmt/core.h"
#include "io/camera.hpp"
#include "tasks/yolo.hpp"
#include "tools/img_tools.hpp"
#include "tools/pnp_check.hpp"

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
// 点序：左上、右上、右下、左下（自左上顺时针），与 Armor::points 一致。
// 注意 tasks/armor.hpp 里的注释写的是「左上、左下、右下、右上」，那是错的 ——
// 见 docs/keypoint_order.md。
static const std::vector<cv::Point3f> object_points{
    {-ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2, 0}, // 点 1 左上
    {ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2, 0},  // 点 2 右上
    {ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2, 0},   // 点 3 右下
    {-ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2, 0}   // 点 4 左下
};
// #########################################################

int main(int argc, char *argv[])
{
    auto_aim::YOLO detector("configs/yolo.yaml");
    io::Camera camera("configs/yolo.yaml");

    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;

    while (true)
    {
        camera.read(img, timestamp);
        if (img.empty())
            break;

        auto armors = detector.detect(img);

        if (!armors.empty())
        {
            auto armor = armors.front();
            tools::draw_points(img, armor.points);

            // #### Task 02 ############################################
            std::vector<cv::Point2f> img_points{
                armor.points.at(0),  // 左上
                armor.points.at(1),  // 右上
                armor.points.at(2),  // 右下
                armor.points.at(3)}; // 左下
            // #########################################################

            // #### Task 03 ############################################
            cv::Mat rvec, tvec;
            cv::solvePnP(object_points, img_points, camera_matrix, distort_coeffs, rvec, tvec);
            // #########################################################

            // #### Task 04 ############################################
            tools::draw_text(img, fmt::format("tvec:  x{: .2f} y{: .2f} z{: .2f}", tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)), cv::Point(10, 60), cv::Scalar(0, 255, 255), 1.7, 3);
            tools::draw_text(img, fmt::format("rvec:  x{: .2f} y{: .2f} z{: .2f}", rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2)), cv::Point(10, 120), cv::Scalar(0, 255, 255), 1.7, 3);
            // #########################################################

            // #### Task 05 ############################################
            cv::Mat rmat;
            cv::Rodrigues(rvec, rmat);
            double yaw = std::atan2(rmat.at<double>(0, 2), rmat.at<double>(2, 2));
            double pitch = -std::asin(rmat.at<double>(1, 2));
            double roll = std::atan2(rmat.at<double>(1, 0), rmat.at<double>(1, 1));
            tools::draw_text(img, fmt::format("euler angles:  yaw{: .2f} pitch{: .2f} roll{: .2f}", yaw, pitch, roll), cv::Point(10, 180), cv::Scalar(0, 255, 255), 1.7, 3);
            // #########################################################

            // 额外的自查显示（不属学生任务）：顺序写对时约 2~3 px
            double reproj = tools::reprojection_error(
                object_points, img_points, rvec, tvec, camera_matrix, distort_coeffs);
            tools::draw_text(img, fmt::format("reproj err:  {:.2f} px", reproj), cv::Point(10, 240), cv::Scalar(0, 255, 255), 1.7, 3);
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
