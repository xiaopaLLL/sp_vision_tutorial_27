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
  auto_aim::Target target;
  std::chrono::steady_clock::time_point tStart, last_fire;
  const double L_x = 0.171, L_y = 0.0;
  int to_fire = 1;  //zero when manual
  int fired = 0;

  while (!exiter.exit()) {
    // Your code start

    // get bullet speed
    auto state = gimbal.state();
    float v0 = (state.bullet_speed > 10 && state.bullet_speed < 30) ? state.bullet_speed : 23.67;

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
    if (key == 'f') {  // press f to fire
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
      // use kalman filtering to predict and update the armor
      if (target.convergened()) target.predict(t);
      if (target_armor != armors.end()) target.update(*target_armor);

      // estimate fly time for the first time
      double d =
        std::sqrt(target.ekf_x()[0] * target.ekf_x()[0] + target.ekf_x()[2] * target.ekf_x()[2]) -
        target.ekf_x()[8];
      double h = target.ekf_x()[4];
      double aim_pitch, fly_time;
      tools::solve_actual_trajectory(v0, d, h, 3, L_x, L_y, aim_pitch, fly_time);

      // iterate fly time and pitch
      double best_id = 0;
      double best_facing = 1e9;
      std::vector<Eigen::Vector4d> xyza_list;
      Eigen::VectorXd predicted_ekf_x;
      const int iter = 5;
      for (int i = 0; i < iter; i++) {
        auto_aim::Target predicted_target = target;
        double latency =
          tools::delta_time(std::chrono::steady_clock::now(), t);  //add a latency when calculated
        predicted_target.predict(latency + fly_time);  //predict the position target will be at
        predicted_ekf_x = predicted_target.ekf_x();
        // get the best armor (minimal angle error)
        xyza_list = predicted_target.armor_xyza_list();
        best_facing = 1e9;
        for (int j = 0; j < xyza_list.size(); j++) {
          double facing = std::abs(
            tools::limit_rad(xyza_list[j][3] - std::atan2(predicted_ekf_x[2], predicted_ekf_x[0])));
          if (facing < best_facing) {
            best_facing = facing;
            best_id = j;
          }
        }
        d = std::sqrt(
          xyza_list[best_id][0] * xyza_list[best_id][0] +
          xyza_list[best_id][1] * xyza_list[best_id][1]);
        h = xyza_list[best_id][2];
        tools::solve_actual_trajectory(v0, d, h, 3, L_x, L_y, aim_pitch, fly_time);
      }
      auto aim_yaw = std::atan2(xyza_list[best_id][1], xyza_list[best_id][0]);

      // send message to gimbal
      float send_yaw = aim_yaw;
      float send_pitch = -aim_pitch;  // send pitch is opposite to real pitch
      // fire only when the armor facing the gimbal enough
      if (to_fire > 0 && tools::delta_time(t, last_fire) > 0.7 && best_facing < 0.04) {
        gimbal.send(true, true, send_yaw, send_pitch);
        // to_fire--; // commented in infinite & auto fire mode
        fired++;
        last_fire = t;
      } else {
        gimbal.send(true, false, send_yaw, send_pitch);
      }

      // plot the results
      nlohmann::json j;
      j["time"] = tools::delta_time(t, tStart);
      j["send_yaw"] = send_yaw;
      j["send_pitch"] = -send_pitch;
      j["targ_x"] = xyza_list[best_id][0];
      j["targ_y"] = xyza_list[best_id][1];
      j["targ_z"] = xyza_list[best_id][2];
      j["r"] = predicted_ekf_x[8];
      j["l"] = predicted_ekf_x[9];
      j["angle"] = predicted_ekf_x[6];
      j["w"] = predicted_ekf_x[7];
      j["fired"] = fired;
      j["d"] = d;
      j["h"] = h;
      j["v0"] = v0;
      j["z"] = predicted_ekf_x[4];
      plotter.plot(j);
    }

    // Your code end
  }

  return 0;
}