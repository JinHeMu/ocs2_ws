// =============================================================================
//  tracer_jaka_joy_whole_body_node.cpp
//
//  通过游戏手柄 (sensor_msgs/Joy) 控制移动底盘的全身轨迹参考节点。
//
//  核心设计: "胡萝卜在棍子上" (carrot-on-a-stick)
//    手柄输出的是速度指令 (v, w), 但 MPC 需要位置目标。
//    为避免积分累积导致目标无限漂移, 目标位姿始终基于当前机器人状态计算:
//
//      desiredX   = curX   + v * cos(curYaw) * lookahead_time
//      desiredY   = curY   + v * sin(curYaw) * lookahead_time
//      desiredYaw = curYaw + w * lookahead_time
//
//    多航点轨迹显式编码速度:
//      在 [leadTime, horizon] 之间均匀插入 N 个航点,
//      每个航点 = 当前位置 + 速度 * dt_i, 让 MPC 清楚看到期望的运动速度。
//
//  松开 LB 时的行为 (publishHoldTarget):
//    底盘: 保持当前位姿不动
//    手臂: 回到 home 位姿 (需要 MPC 主动施力对抗重力, 不会下坠)
//
//  控制映射:
//    左摇杆 Y (axis 1)  → 底盘前进/后退线速度
//    右摇杆 X (axis 3)  → 底盘转向角速度
//    LB (button 4)      → 死区按钮
//    A  (button 0)      → 机械臂切回 home 位姿
//    B  (button 1)      → 机械臂保持当前构型
// =============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <ocs2_msgs/msg/mpc_observation.hpp>

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

class TracerJakaJoyWholeBodyNode : public rclcpp::Node {
 public:
  TracerJakaJoyWholeBodyNode() : Node("tracer_jaka_joy_whole_body_node") {
    // ── 基础参数 ──────────────────────────────────────────────
    declare_parameter<std::string>("robot_name",   "mobile_manipulator");
    declare_parameter<std::string>("world_frame",  "odom");
    declare_parameter<double>("publish_rate",      50.0);
    declare_parameter<double>("linear_speed_max",  0.4);
    declare_parameter<double>("angular_speed_max", 1.0);
    declare_parameter<double>("deadzone",          0.10);
    // lookahead_time: "胡萝卜" 放在前方多远 [s]
    //   越大 → 目标越远, MPC 速度跟踪越激进
    //   越小 → 目标越近, 更保守但可能速度起不来
    declare_parameter<double>("lookahead_time",    1.5);
    declare_parameter<double>("trajectory_horizon", 2.0);
    declare_parameter<double>("lead_time",         0.05);
    // num_waypoints: 轨迹航点数 (含首尾, ≥2)
    //   越多 → 速度曲线编码越精细, 但 MPC 跟踪 cost 计算越重
    declare_parameter<int>("num_waypoints",        5);
    declare_parameter<int>("state_dim",            9);
    declare_parameter<int>("input_dim",            8);
    declare_parameter<int>("base_dim",             3);
    declare_parameter<int>("arm_dim",              6);

    // ── 手柄轴/按键映射 ──────────────────────────────────────
    declare_parameter<std::string>("joy_topic",  "/joy");
    declare_parameter<int>("axis_linear",        1);    // 左摇杆 Y
    declare_parameter<int>("axis_angular",       3);    // 右摇杆 X
    declare_parameter<int>("button_deadman",     4);    // LB
    declare_parameter<int>("button_arm_home",    0);    // A
    declare_parameter<int>("button_arm_hold",    1);    // B

    // ── 机械臂 home 位姿 (与 task.info 初始状态一致) ────────
    declare_parameter<std::vector<double>>(
        "arm_home",
        std::vector<double>{-0.515, 1.5707, -1.5707, 1.5707, 1.5707, 0.254});

    // ── 读取参数 ──────────────────────────────────────────────
    robotName_     = get_parameter("robot_name").as_string();
    worldFrame_    = get_parameter("world_frame").as_string();
    publishRate_   = get_parameter("publish_rate").as_double();
    vMax_          = get_parameter("linear_speed_max").as_double();
    wMax_          = get_parameter("angular_speed_max").as_double();
    deadzone_      = get_parameter("deadzone").as_double();
    lookaheadTime_ = get_parameter("lookahead_time").as_double();
    horizon_       = get_parameter("trajectory_horizon").as_double();
    leadTime_      = get_parameter("lead_time").as_double();
    numWaypoints_  = get_parameter("num_waypoints").as_int();
    stateDim_      = get_parameter("state_dim").as_int();
    inputDim_      = get_parameter("input_dim").as_int();
    baseDim_       = get_parameter("base_dim").as_int();
    armDim_        = get_parameter("arm_dim").as_int();

    axLinear_   = get_parameter("axis_linear").as_int();
    axAngular_  = get_parameter("axis_angular").as_int();
    btnDeadman_ = get_parameter("button_deadman").as_int();
    btnArmHome_ = get_parameter("button_arm_home").as_int();
    btnArmHold_ = get_parameter("button_arm_hold").as_int();

    const auto homeVec = get_parameter("arm_home").as_double_array();
    armHome_.resize(armDim_);
    for (int i = 0; i < armDim_ && i < static_cast<int>(homeVec.size()); ++i) {
      armHome_(i) = homeVec[i];
    }

    // 参数保护
    if (baseDim_ + armDim_ != stateDim_) {
      RCLCPP_FATAL(get_logger(),
                   "base_dim(%d) + arm_dim(%d) != state_dim(%d)",
                   baseDim_, armDim_, stateDim_);
      throw std::runtime_error("dimension mismatch");
    }
    if (publishRate_ <= 1e-6) { publishRate_ = 50.0; }
    if (lookaheadTime_ <= 0.05) { lookaheadTime_ = 1.0; }
    if (horizon_ <= lookaheadTime_ + 0.1) { horizon_ = lookaheadTime_ + 0.5; }
    if (numWaypoints_ < 2) { numWaypoints_ = 2; }
    if (numWaypoints_ > 20) { numWaypoints_ = 20; }

    // ── 订阅 Joy ──────────────────────────────────────────────
    const auto joyTopic = get_parameter("joy_topic").as_string();
    joySub_ = create_subscription<sensor_msgs::msg::Joy>(
        joyTopic, rclcpp::SensorDataQoS(),
        std::bind(&TracerJakaJoyWholeBodyNode::onJoy, this,
                  std::placeholders::_1));

    // ── 订阅 MPC observation (拿时间 + 当前全身状态) ─────────
    const std::string obsTopic = robotName_ + "_mpc_observation";
    obsSub_ = create_subscription<ocs2_msgs::msg::MpcObservation>(
        obsTopic, rclcpp::QoS(1),
        [this](ocs2_msgs::msg::MpcObservation::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lk(mtx_);
          latestObsTime_ = msg->time;
          const auto& v = msg->state.value;
          if (static_cast<int>(v.size()) == stateDim_) {
            currentState_.resize(stateDim_);
            for (int i = 0; i < stateDim_; ++i) {
              currentState_(i) = static_cast<double>(v[i]);
            }
            haveState_ = true;
          }
          haveObs_ = true;
        });

    RCLCPP_INFO(get_logger(),
                "[JoyWholeBody] 订阅 joy: %s, observation: %s",
                joyTopic.c_str(), obsTopic.c_str());
  }

  void init() {
    targetPub_ = std::make_unique<ocs2::TargetTrajectoriesRosPublisher>(
        shared_from_this(), robotName_);

    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publishRate_));

    timer_ = create_wall_timer(
        period_ns,
        std::bind(&TracerJakaJoyWholeBodyNode::onTimer, this));

    RCLCPP_INFO(get_logger(),
                "[JoyWholeBody] 就绪. rate=%.1fHz vMax=%.2f wMax=%.2f "
                "lookahead=%.2fs horizon=%.2fs waypoints=%d deadzone=%.2f",
                publishRate_, vMax_, wMax_, lookaheadTime_, horizon_,
                numWaypoints_, deadzone_);
    RCLCPP_INFO(get_logger(),
                "[JoyWholeBody] 左摇杆Y=axis%d→前进, 右摇杆X=axis%d→转向, "
                "LB=btn%d 安全开关",
                axLinear_, axAngular_, btnDeadman_);
    RCLCPP_INFO(get_logger(),
                "[JoyWholeBody] A=btn%d 臂归home, B=btn%d 臂保持当前",
                btnArmHome_, btnArmHold_);
    RCLCPP_INFO(get_logger(),
                "[JoyWholeBody] 松开LB时: 底盘保持位置, 手臂→home (MPC主动对抗重力)");
  }

 private:
  // =====================================================================
  // 手柄工具
  // =====================================================================

  double applyDeadzone(double v) const {
    if (std::abs(v) < deadzone_) { return 0.0; }
    const double sign = (v > 0.0) ? 1.0 : -1.0;
    return sign * (std::abs(v) - deadzone_) / (1.0 - deadzone_);
  }

  double getAxis(const sensor_msgs::msg::Joy& j, int idx) const {
    if (idx < 0 || idx >= static_cast<int>(j.axes.size())) { return 0.0; }
    return applyDeadzone(j.axes[idx]);
  }

  bool getButton(const sensor_msgs::msg::Joy& j, int idx) const {
    if (idx < 0 || idx >= static_cast<int>(j.buttons.size())) { return false; }
    return j.buttons[idx] != 0;
  }

  // =====================================================================
  // 构建多航点速度轨迹 (胡萝卜模式)
  //
  // 在 [leadTime, horizon] 之间均匀插入 N 个航点:
  //   waypoint_i = curState + (v, w) * (leadTime + i*dt)
  // 显式编码期望速度, 让 MPC 的 WholeBodyTrajectoryCost 明确知道要跑多快。
  // =====================================================================

  ocs2::TargetTrajectories buildCarrotTrajectory(
      double obsTime, const ocs2::vector_t& curState,
      double v, double w, const ocs2::vector_t& armTarget) const {

    const double curYaw = curState(2);
    const double dt = (horizon_ - leadTime_) / std::max(1, numWaypoints_ - 1);

    ocs2::scalar_array_t timeTraj;
    ocs2::vector_array_t stateTraj;
    ocs2::vector_array_t inputTraj;

    timeTraj.reserve(numWaypoints_);
    stateTraj.reserve(numWaypoints_);
    inputTraj.reserve(numWaypoints_);

    for (int i = 0; i < numWaypoints_; ++i) {
      const double t = leadTime_ + i * dt;   // 相对 obsTime 的时间偏移

      ocs2::vector_t s = ocs2::vector_t::Zero(stateDim_);
      s(0) = curState(0) + v * std::cos(curYaw) * t;
      s(1) = curState(1) + v * std::sin(curYaw) * t;
      s(2) = curYaw      + w * t;
      for (int j = 0; j < armDim_; ++j) {
        s(baseDim_ + j) = armTarget(j);
      }

      timeTraj.push_back(obsTime + t);
      stateTraj.push_back(s);
      inputTraj.push_back(ocs2::vector_t::Zero(inputDim_));
    }

    return ocs2::TargetTrajectories(
        std::move(timeTraj), std::move(stateTraj), std::move(inputTraj));
  }

  // =====================================================================
  // 静止保持轨迹 (松开 LB 时)
  //
  // 底盘: 保持在当前位置不动 (首尾航点 base 部分相同)
  // 手臂: 从当前构型平滑过渡到 home, 让 MPC 主动施力对抗重力
  // =====================================================================

  ocs2::TargetTrajectories buildHoldTrajectory(
      double obsTime, const ocs2::vector_t& curState) const {

    ocs2::vector_t holdState = curState;
    for (int i = 0; i < armDim_; ++i) {
      holdState(baseDim_ + i) = armHome_(i);
    }

    ocs2::scalar_array_t timeTraj;
    ocs2::vector_array_t stateTraj;
    ocs2::vector_array_t inputTraj;

    // waypoint 0: 当前状态 (给 MPC 连续的起点)
    timeTraj.push_back(obsTime + leadTime_);
    stateTraj.push_back(curState);
    inputTraj.push_back(ocs2::vector_t::Zero(inputDim_));

    // waypoint 1: 保持位姿, 手臂回到 home
    //   底盘部分 = 当前底盘位姿 (不动)
    //   手臂部分 = home (需要 MPC 对抗重力)
    timeTraj.push_back(obsTime + horizon_);
    stateTraj.push_back(holdState);
    inputTraj.push_back(ocs2::vector_t::Zero(inputDim_));

    return ocs2::TargetTrajectories(
        std::move(timeTraj), std::move(stateTraj), std::move(inputTraj));
  }

  // =====================================================================
  // 回调
  // =====================================================================

  void onJoy(sensor_msgs::msg::Joy::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    lastJoy_ = std::move(msg);
  }

  void onTimer() {
    sensor_msgs::msg::Joy::ConstSharedPtr joy;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      joy = lastJoy_;
    }
    if (!joy) { return; }

    double obsTime;
    ocs2::vector_t curState;
    bool haveState;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      obsTime   = latestObsTime_;
      curState  = currentState_;
      haveState = haveState_;
    }
    if (!haveState) { return; }

    // ── 按钮: 臂归 home / 保持当前 ────────────────────────────
    const bool armHome = getButton(*joy, btnArmHome_);
    const bool armHold = getButton(*joy, btnArmHold_);

    if (armHome && !lastArmHome_) {
      useArmHome_ = true;
      RCLCPP_INFO(get_logger(), "[JoyWholeBody] 臂目标 → home");
    }
    if (armHold && !lastArmHold_) {
      useArmHome_ = false;
      RCLCPP_INFO(get_logger(), "[JoyWholeBody] 臂目标 → 保持当前构型");
    }
    lastArmHome_ = armHome;
    lastArmHold_ = armHold;

    // ── deadman ───────────────────────────────────────────────
    const bool deadman = getButton(*joy, btnDeadman_);
    if (!deadman) {
      // ★ 关键修复: 底盘保持位置, 手臂→home (MPC 需要施力对抗重力)
      if (wasDeadman_) {
        // 刚松开的那一帧发一次保持轨迹
        targetPub_->publishTargetTrajectories(
            buildHoldTrajectory(obsTime, curState));
        RCLCPP_DEBUG(get_logger(),
                     "[JoyWholeBody] LB 松开, 发送保持轨迹 (臂→home)");
      }
      wasDeadman_ = false;
      return;
    }
    wasDeadman_ = true;

    // ── 摇杆输入 ──────────────────────────────────────────────
    const double stickLin = getAxis(*joy, axLinear_);
    const double stickAng = getAxis(*joy, axAngular_);

    const double v = stickLin * vMax_;
    const double w = stickAng * wMax_;

    // ── 机械臂目标 ─────────────────────────────────────────────
    ocs2::vector_t armTarget = armHome_;
    if (!useArmHome_) {
      armTarget.resize(armDim_);
      for (int i = 0; i < armDim_; ++i) {
        armTarget(i) = curState(baseDim_ + i);
      }
    }

    // ── 发布多航点速度轨迹 ────────────────────────────────────
    targetPub_->publishTargetTrajectories(
        buildCarrotTrajectory(obsTime, curState, v, w, armTarget));
  }

  // =====================================================================
  // 成员变量
  // =====================================================================

  std::string robotName_, worldFrame_;

  int stateDim_{9}, inputDim_{8}, baseDim_{3}, armDim_{6};
  int numWaypoints_{5};

  double publishRate_{50.0};
  double vMax_{0.4}, wMax_{1.0};
  double deadzone_{0.10};
  double lookaheadTime_{1.5}, horizon_{2.0}, leadTime_{0.05};

  int axLinear_{1}, axAngular_{3};
  int btnDeadman_{4}, btnArmHome_{0}, btnArmHold_{1};

  ocs2::vector_t armHome_;
  bool useArmHome_{true};

  // 按钮上升沿检测
  bool lastArmHome_{false}, lastArmHold_{false};
  bool wasDeadman_{false};   // 上一帧 deadman 状态

  // 订阅
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySub_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr obsSub_;

  // 发布
  std::unique_ptr<ocs2::TargetTrajectoriesRosPublisher> targetPub_;

  // 定时器
  rclcpp::TimerBase::SharedPtr timer_;

  // 线程安全
  std::mutex mtx_;
  sensor_msgs::msg::Joy::ConstSharedPtr lastJoy_;
  double latestObsTime_{0.0};
  ocs2::vector_t currentState_;
  bool haveState_{false}, haveObs_{false};
};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TracerJakaJoyWholeBodyNode>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
