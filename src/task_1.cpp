#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  // 初始化工具类
  tools::Exiter exiter;
  tools::Plotter plotter;  // 注意plotter工具的使用

  // 初始化io类
  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);

  // 初始化auto_aim类
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  while (!exiter.exit()) {
    // Your code start

    // record the start time of this loop iteration for tools::delta_time
    std::chrono::steady_clock::time_point tStart;

    // try read a frame from camera
    if (!camera.try_read_for(img, t, 100ms)) {
      tools::logger()->warn("Failed to read image from camera!");
      continue;
    }

    // get the gimbal's quaternion at the time of image capture
    q = gimbal.q(t);
    solver.set_R_gimbal2world(q);

    // show the image (done in yolo so only waitKey)
    if (cv::waitKey(1) == 'q') {
      break;
    }

    // detect armors in the image using YOLO
    std::list<auto_aim::Armor> armors = yolo.detect(img);
    if (armors.empty()) {
      continue;
    }

    // solve the pose of each detected armor
    for (auto & armor : armors) {
      solver.solve(armor);
    }

    // choose the armor with the minimum distance as target
    auto target_armor = std::min_element(
      armors.begin(), armors.end(), [](const auto_aim::Armor & a, const auto_aim::Armor & b) {
        return a.ypd_in_world[2] < b.ypd_in_world[2];
      });

    // send the gimbal command to aim at target
    float target_yaw = target_armor->ypd_in_world[0];
    float target_pitch = target_armor->ypd_in_world[1];
    gimbal.send(
      true, false, target_yaw,
      -target_pitch);  // gimbal's receiving pitch is opposite to solver result

    // plot the results
    nlohmann::json j;
    j["time"] = tools::delta_time(t, tStart);
    j["yaw"] = target_yaw;
    j["pitch"] = target_pitch;
    plotter.plot(j);

    // Your code end
  }

  return 0;
}