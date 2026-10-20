#include <chrono>
#include <opencv2/opencv.hpp>
#include <tools/trajectory.hpp>  // extra included

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/target.hpp"
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

  // my definitions
  auto_aim::Target target;  // target class
  std::chrono::steady_clock::time_point tStart, last_fire;
  const double L_x = 0.171, L_y = 0.0;
  int to_fire = 0;
  int fired = 0;
  bool can_fire = false;

  while (!exiter.exit()) {
    // Your code start
    auto state = gimbal.state();  // bullet speed
    float v0 = (state.bullet_speed > 10 && state.bullet_speed < 30) ? state.bullet_speed : 23.67;
    // std::cout << v0 << std::endl;
    // v0 = 20;

    // try read a frame from camera
    if (!camera.try_read_for(img, t, 100ms)) {
      tools::logger()->warn("Failed to read image from camera!");
      continue;
    }

    // get the gimbal's quaternion at the time of image capture
    q = gimbal.q(t);
    solver.set_R_gimbal2world(q);

    // show the image (done in yolo so only waitKey)
    const int key = cv::waitKey(1);
    if (key == 'q') break;
    if (key == 'f') {
      can_fire = true;
      to_fire = 3;
      std::cout << "to_fire: " << to_fire << std::endl;
    }

    // detect and solve armors
    std::list<auto_aim::Armor> armors = yolo.detect(img);
    for (auto & armor : armors) solver.solve(armor);

    // choose the armor with the minimum distance as target
    auto target_armor = std::min_element(
      armors.begin(), armors.end(), [](const auto_aim::Armor & a, const auto_aim::Armor & b) {
        return a.ypd_in_world[2] < b.ypd_in_world[2];
      });

    // initialize target when first armor detected
    if (!target.isinit && target_armor != armors.end()) {
      target = auto_aim::Target(*target_armor, t, 0.2, 4);
      tStart = std::chrono::steady_clock::now();
      last_fire = tStart;
    }

    if (target.isinit) {
      if (target.convergened()) target.predict(t);
      if (target_armor != armors.end()) target.update(*target_armor);

      // estimate fly time
      Eigen::Vector4d target_xyza = target.armor_xyza_list()[0];
      for (const Eigen::Vector4d & xyza : target.armor_xyza_list()) {
        if (tools::xyz2ypd(xyza.head(3))[2] < tools::xyz2ypd(target_xyza.head(3))[2])
          target_xyza = xyza;
      }
      double d = std::sqrt(target_xyza[0] * target_xyza[0] + target_xyza[1] * target_xyza[1]);
      double h = target_xyza[2];
      double fly_time;
      double aim_pitch;
      if (!tools::solve_actual_trajectory(v0, d, h, 3, L_x, L_y, aim_pitch, fly_time)) {
        tools::logger()->warn("Failed to solve actual trajectory!");
        continue;
      }

      // predict the position after fly_time (considering gravity influence)
      // fly_time = 0;
      auto_aim::Target predicted_target = target;
      double latency = tools::delta_time(std::chrono::steady_clock::now(), t);
      predicted_target.predict(latency + fly_time);

      // choose the closest armor as the target armor to aim
      double best_facing = 1e9;
      Eigen::Vector4d predicted_target_xyza = predicted_target.armor_xyza_list()[0];
      for (const Eigen::Vector4d & xyza : predicted_target.armor_xyza_list()) {
        double facing = std::abs(
          tools::limit_rad(
            xyza[3] - std::atan2(predicted_target.ekf_x()[2], predicted_target.ekf_x()[0])));
        if (facing < best_facing) {
          best_facing = facing;
          predicted_target_xyza = xyza;
        }
      }

      // solve the yaw and pitch to aim at the target armor
      double aim_yaw = std::atan2(predicted_target_xyza[1], predicted_target_xyza[0]);
      d = std::sqrt(
        predicted_target_xyza[0] * predicted_target_xyza[0] +
        predicted_target_xyza[1] * predicted_target_xyza[1]);
      h = predicted_target_xyza[2];
      if (!tools::solve_actual_trajectory(v0, d, h, 3, L_x, L_y, aim_pitch, fly_time)) {
        tools::logger()->warn("Failed to solve actual trajectory!");
        continue;
      }

      float send_yaw = aim_yaw;
      float send_pitch = -aim_pitch;  // send pitch is opposite to real pitch
      // std::cout << send_yaw << ' ' << send_pitch << std::endl;

      // send message to gimbal
      if (predicted_target.convergened()) {
        // fire only when the armor facing the gimbal enough
        if (
          can_fire && to_fire > 0 && tools::delta_time(t, last_fire) > 1.5 && best_facing < 0.1) {
          gimbal.send(true, true, send_yaw, send_pitch);
          to_fire--;
          fired++;
          last_fire = t;
        } else {
          gimbal.send(true, false, send_yaw, send_pitch);
        }
      } else {
        gimbal.send(false, false, 0, 0);
      }

      // plot the results
      nlohmann::json j;
      j["time"] = tools::delta_time(t, tStart);
      j["yaw"] = send_yaw;
      j["pitch"] = send_pitch;
      j["targ_x"] = target_xyza[0];
      j["targ_y"] = target_xyza[1];
      j["targ_z"] = target_xyza[2];
      j["r"] = predicted_target.ekf_x()[8];
      j["l"] = predicted_target.ekf_x()[9];
      j["angle"] = predicted_target.ekf_x()[6];
      j["w"] = predicted_target.ekf_x()[7];
      j["fired"] = fired;
      j["d"] = std::sqrt(
        predicted_target.ekf_x()[0] * predicted_target.ekf_x()[0] +
        predicted_target.ekf_x()[2] * predicted_target.ekf_x()[2]);
      j["h"] = predicted_target.ekf_x()[10];
      j["v0"] = v0;
      j["z"] = predicted_target.ekf_x()[4];
      plotter.plot(j);
    }

    // Your code end
  }

  return 0;
}