// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "jaka_hardware_interface/jaka_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <thread>
// 确保包含 Eigen 头文件（若头文件中已包含可忽略）
#include <Eigen/Dense>

using namespace std;

namespace jaka_hardware_interface
{
  hardware_interface::CallbackReturn JakaHardwareInterface::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    // 1. 获取 Robot IP
    auto it = info_.hardware_parameters.find("robot_ip");
    if (it != info_.hardware_parameters.end())
    {
      robot_ip_ = it->second;
    }
    else
    {
      RCLCPP_FATAL(rclcpp::get_logger("JakaHardwareInterface"), "Parameter'robot_ip' not set");
      return CallbackReturn::ERROR;
    }

    // 2. 获取 Local IP (EDG 必需)
    auto it_local = info_.hardware_parameters.find("local_ip");
    if (it_local != info_.hardware_parameters.end())
    {
      local_ip_ = it_local->second;
    }
    else
    {
      RCLCPP_FATAL(rclcpp::get_logger("JakaHardwareInterface"), "Parameter'local_ip' not set (Required for EDG)");
      return CallbackReturn::ERROR;
    }

    // 3. 获取力传感器偏置初始值（后续将在 on_activate 中被动态零偏覆盖）
    ft_bias_.resize(6, 0.0);
    std::vector<std::string> bias_keys = {"ft_bias_fx", "ft_bias_fy", "ft_bias_fz", "ft_bias_tx", "ft_bias_ty", "ft_bias_tz"};
    std::vector<double> default_biases = {-9.80, -6.78, -6.00, 0.54, -0.63, 0.03};

    for (size_t i = 0; i < bias_keys.size(); ++i)
    {
      auto it_bias = info_.hardware_parameters.find(bias_keys[i]);
      ft_bias_[i] = (it_bias != info_.hardware_parameters.end()) ? std::stod(it_bias->second) : default_biases[i];
    }

    // 4. 获取力臂向量参数 (r_t_s)
    r_t_s_.resize(3, 0.0);
    std::vector<std::string> r_t_s_keys = {"r_t_s_x", "r_t_s_y", "r_t_s_z"};
    std::vector<double> default_r_t_s = {0.000, -0.05, -0.4}; // 默认值

    for (size_t i = 0; i < r_t_s_keys.size(); ++i)
    {
      auto it_r = info_.hardware_parameters.find(r_t_s_keys[i]);
      r_t_s_[i] = (it_r != info_.hardware_parameters.end()) ? std::stod(it_r->second) : default_r_t_s[i];
    }

    // 5. 获取死区阈值 (ft_deadband_force, ft_deadband_torque)
    auto it_db_f = info_.hardware_parameters.find("ft_deadband_force");
    deadband_force_ = (it_db_f != info_.hardware_parameters.end()) ? std::stod(it_db_f->second) : 1.0;

    auto it_db_t = info_.hardware_parameters.find("ft_deadband_torque");
    deadband_torque_ = (it_db_t != info_.hardware_parameters.end()) ? std::stod(it_db_t->second) : 0.2;

    // 6. 获取滤波系数
    auto it_alpha = info_.hardware_parameters.find("ft_filter_alpha");
    filter_alpha_ = (it_alpha != info_.hardware_parameters.end()) ? std::stod(it_alpha->second) : 0.2;

    RCLCPP_INFO(rclcpp::get_logger("JakaHardwareInterface"),
                "FTS Params Loaded - Deadband: [F:%.2f, T:%.2f], Alpha: %.2f",
                deadband_force_, deadband_torque_, filter_alpha_);
                
    RCLCPP_INFO(rclcpp::get_logger("JakaHardwareInterface"),
                "Force Arm Vector Loaded: [%.3f, %.3f, %.3f]",
                r_t_s_[0], r_t_s_[1], r_t_s_[2]);

    hw_position_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_velocity_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_position_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    hw_fts_states_.resize(6, 0.0); // 输出给ROS的数据 (初始为0)
    hw_fts_raw_.resize(6, 0.0);    // 原始数据容器

    RCLCPP_INFO(rclcpp::get_logger("JakaHardwareInterface"),
                "Jaka EDG Interface Init: Robot=%s, Local=%s", robot_ip_.c_str(), local_ip_.c_str());

    return CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn JakaHardwareInterface::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("JakaHardwareInterface"), "Connecting to robot...");

    // 登录
    if (robot_.login_in(robot_ip_.c_str()) != ERR_SUCC)
    {
      RCLCPP_ERROR(rclcpp::get_logger("JakaHardwareInterface"), "Login failed!");
      return CallbackReturn::ERROR;
    }

    // 初始化 EDG 模式
    RCLCPP_INFO(rclcpp::get_logger("JakaHardwareInterface"), "Initializing EDG UDP Stream...");
    if (robot_.edg_init(true, local_ip_.c_str(), 10010, 0) != ERR_SUCC)
    {
      RCLCPP_ERROR(rclcpp::get_logger("JakaHardwareInterface"), "Failed to init EDG! Check firewall/IP.");
      return CallbackReturn::ERROR;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 读取初始状态 (使用 EDG 接口)
    int retry_count = 0;
    while (true)
    {
      if (robot_.edg_get_stat(&edg_state_) == ERR_SUCC)
      {
        break;
      }
      retry_count++;
      if (retry_count > 20)
      {
        RCLCPP_ERROR(rclcpp::get_logger("JakaHardwareInterface"), "Timeout waiting for initial EDG data.");
        return CallbackReturn::ERROR;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 同步 Command 和 State，防止启动飞车
    for (size_t i = 0; i < info_.joints.size() && i < 6; ++i)
    {
      hw_position_states_[i] = edg_state_.jointVal.jVal[i];
      hw_velocity_states_[i] = edg_state_.jointVel.jVel[i]; // rad/s
      hw_position_commands_[i] = hw_position_states_[i]; // 初始指令 = 当前位置
      RCLCPP_INFO(rclcpp::get_logger("JakaHardwareInterface"), "Joint %zu init pos: %.4f", i, hw_position_states_[i]);
    }

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> JakaHardwareInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_states_[i]));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_states_[i]));
    }

    if (info_.sensors.size() > 0)
    {
      const auto &sensor = info_.sensors[0]; 

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          sensor.name, "force.x", &hw_fts_states_[0]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          sensor.name, "force.y", &hw_fts_states_[1]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          sensor.name, "force.z", &hw_fts_states_[2]));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          sensor.name, "torque.x", &hw_fts_states_[3]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          sensor.name, "torque.y", &hw_fts_states_[4]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          sensor.name, "torque.z", &hw_fts_states_[5]));

      RCLCPP_INFO(rclcpp::get_logger("JakaHardwareInterface"),
                  "Exported FT Sensor interfaces for: %s", sensor.name.c_str());
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> JakaHardwareInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_commands_[i]));
    }

    return command_interfaces;
  }

  hardware_interface::CallbackReturn JakaHardwareInterface::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("JakaHardwareInterface"), "Activating... (Ensuring EDG is running)");
    robot_.servo_move_enable(true); // 机器人上电/使能

    std::fill(hw_fts_states_.begin(), hw_fts_states_.end(), 0.0);

    // ================== 上电后动态获取零偏 ==================
    RCLCPP_INFO(rclcpp::get_logger("JakaHardwareInterface"), "Calibrating FT Sensor zero-drift...");
    std::array<double, 6> ft_sum = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    int sample_count = 50;  // 采样 50 次
    int valid_samples = 0;

    for (int i = 0; i < sample_count; ++i)
    {
      if (robot_.edg_get_stat(&edg_state_) == ERR_SUCC)
      {
        ft_sum[0] += edg_state_.torqSensor.fx;
        ft_sum[1] += edg_state_.torqSensor.fy;
        ft_sum[2] += edg_state_.torqSensor.fz;
        ft_sum[3] += edg_state_.torqSensor.tx;
        ft_sum[4] += edg_state_.torqSensor.ty;
        ft_sum[5] += edg_state_.torqSensor.tz;
        valid_samples++;
      }
      // 等待 10ms，总采样时间约 500ms
      std::this_thread::sleep_for(std::chrono::milliseconds(10)); 
    }

    if (valid_samples > 0)
    {
      // 更新动态零偏，覆盖从参数文件读入的默认零偏
      for (int i = 0; i < 6; ++i)
      {
        ft_bias_[i] = ft_sum[i] / valid_samples;
      }
      RCLCPP_INFO(rclcpp::get_logger("JakaHardwareInterface"),
                  "FT Calibration Success! Calculated Bias: [F:%.2f, %.2f, %.2f, T:%.2f, %.2f, %.2f]",
                  ft_bias_[0], ft_bias_[1], ft_bias_[2], ft_bias_[3], ft_bias_[4], ft_bias_[5]);
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("JakaHardwareInterface"),
                  "FT Calibration failed! Continuing with parameter-based defaults.");
    }
    // ==========================================================

    // 再次同步，因为从 Configure 到 Activate 可能有时间差
    robot_.edg_get_stat(&edg_state_);
    for (size_t i = 0; i < info_.joints.size() && i < 6; ++i)
    {
      hw_position_commands_[i] = edg_state_.jointVal.jVal[i];
    }

    return CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn JakaHardwareInterface::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("JakaHardwareInterface"), "Deactivating... Stopping EDG");

    robot_.servo_move_enable(false);
    // 关闭 EDG 模式
    robot_.edg_init(false);

    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type JakaHardwareInterface::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // 使用 EDG 接口读取全量数据
    errno_t ret = robot_.edg_get_stat(&edg_state_);

    if (ret == ERR_SUCC)
    {
      // A. 更新关节状态
      for (size_t i = 0; i < info_.joints.size() && i < 6; ++i)
      {
        hw_position_states_[i] = edg_state_.jointVal.jVal[i];
        hw_velocity_states_[i] = edg_state_.jointVel.jVel[i];
      }

      // B. 更新力传感器数据
      if (hw_fts_states_.size() == 6)
      {
        // 1. 原始力/力矩 (Raw 减去 在on_activate中计算出的动态Bias)
        Eigen::Vector3d F_sensor(edg_state_.torqSensor.fx - ft_bias_[0],
                                 edg_state_.torqSensor.fy - ft_bias_[1],
                                 edg_state_.torqSensor.fz - ft_bias_[2]);
        Eigen::Vector3d M_sensor(edg_state_.torqSensor.tx - ft_bias_[3],
                                 edg_state_.torqSensor.ty - ft_bias_[4],
                                 edg_state_.torqSensor.tz - ft_bias_[5]);

        // 2. 构建旋转矩阵 R_sensor_to_tool
        // TODO: 如果你后续需要，也可以把这个矩阵提取成参数。目前保持硬编码。
        Eigen::Matrix3d R_s_t;
        R_s_t <<  0.707, -0.707,  0.000,
                  0.664,  0.664,  0.342,
                 -0.242, -0.242,  0.940;

        // 3. 构建力臂向量 r (tool0 到 sensor 的位移，由参数初始化)
        Eigen::Vector3d r_t_s(r_t_s_[0], r_t_s_[1], r_t_s_[2]);

        // 4. 物理变换
        // F_tool = R * F_sensor
        Eigen::Vector3d F_tool = R_s_t * F_sensor;

        // M_tool = R * M_sensor + (r x F_tool)
        Eigen::Vector3d M_tool = R_s_t * M_sensor + r_t_s.cross(F_tool);

        // 5. 滤波与输出给导纳控制器
        std::array<double, 6> compensated_ft = {F_tool.x(), F_tool.y(), F_tool.z(),
                                                M_tool.x(), M_tool.y(), M_tool.z()};

        for (int i = 0; i < 6; ++i)
        {
          // 1. 低通滤波 (防止导纳控制震荡)
          hw_fts_states_[i] = filter_alpha_ * compensated_ft[i] + (1.0 - filter_alpha_) * hw_fts_states_[i];

          // 2. 死区处理 (防止静止时的微小漂移触发运动)
          double db = (i < 3) ? deadband_force_ : deadband_torque_;
          if (std::abs(hw_fts_states_[i]) < db)
          {
            hw_fts_states_[i] = 0.0;
          }
        }
      }
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type JakaHardwareInterface::write(
      const rclcpp::Time &, const rclcpp::Duration & /*period*/)
  {
    // 准备指令数据
    for (size_t i = 0; i < info_.joints.size() && i < 6; ++i)
    {
      // NaN 检查
      if (std::isnan(hw_position_commands_[i]))
      {
        joint_cmd_.jVal[i] = hw_position_states_[i];
      }
      else
      {
        joint_cmd_.jVal[i] = hw_position_commands_[i];
      }
    }

    static std::array<double, 6> last_sent{};
    static bool inited = false;

    double eps = 1e-5; // 约0.0001 rad ≈ 0.0057°
    bool changed = false;
    for (int i = 0; i < 6; i++)
    {
      if (!inited || std::fabs(joint_cmd_.jVal[i] - last_sent[i]) > eps)
      {
        changed = true;
      }
    }

    if (changed)
    {
      robot_.edg_servo_j(&joint_cmd_, MoveMode::ABS, 1);
      for (int i = 0; i < 6; i++)
        last_sent[i] = joint_cmd_.jVal[i];
      inited = true;
    }
    
    return hardware_interface::return_type::OK;
  }
} // namespace jaka_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    jaka_hardware_interface::JakaHardwareInterface, hardware_interface::SystemInterface)
