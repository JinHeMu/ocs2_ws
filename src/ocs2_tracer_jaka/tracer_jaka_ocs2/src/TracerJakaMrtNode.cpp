// =============================================================================
//  TracerJakaMrtNode.cpp
//
//  OCS2 MRT 节点 + 控制器桥 (ROS 2)。
//
//  仿真 / 实机共用一份源码，靠参数切换:
//    use_stamped_cmd : true  (默认)   -> 发布 TwistStamped (仿真 diff_drive_controller)
//                      false          -> 发布 Twist        (实机 tracer_base_ros)
//
//  其余设计点 (沿用上一版):
//   * 初始 target (use_whole_body_target=true, 默认) = 当前全身状态 (stateDim 维),
//     MPC 一开始就"保持当前构型", 等待 whole_body_trajectory_target_node 发来全身轨迹
//   * use_whole_body_target=false 时退回旧行为: 初始 target = 当前 EE 位姿 (7 维)
//   * JTC 轨迹只发 position, 不发 velocity, 避开 "last point velocity != 0" 拒收
//   * 单点轨迹 (1 个 point @ traj_horizon ahead), 避免 currentTime > plan_end
//   * MRT 用独立 internal node, 不跟主 node 抢 executor
// =============================================================================

#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_mobile_manipulator/ManipulatorModelInfo.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

using namespace std::chrono_literals;


/** /diff_drive_controller/odom
        ↓
odomCallback()
        ↓
baseX_, baseY_, baseYaw_

/joint_states
        ↓
jsCallback()
        ↓
armQ_

baseX_, baseY_, baseYaw_, armQ_
        ↓
fillStateLocked()
        ↓
ocs2::SystemObservation
        ↓
mrt_->setCurrentObservation(obs)
        ↓
/mobile_manipulator_mpc_observation
        ↓
MPC node 求解
        ↓
/mobile_manipulator_mpc_policy
        ↓
mrt_->spinMRT()
mrt_->updatePolicy()
mrt_->evaluatePolicy()
        ↓
底盘 Twist / TwistStamped
机械臂 JointTrajectory **/


class TracerJakaMrtBridge : public rclcpp::Node {
 public:
  TracerJakaMrtBridge() : Node("tracer_jaka_mrt_node") {
    // ----- params -----
    declare_parameter<std::string>("taskFile",          "");
    declare_parameter<std::string>("libFolder",         "");
    declare_parameter<std::string>("urdfFile",          "");
    declare_parameter<double>("mrt_loop_rate",          100.0);
    declare_parameter<double>("traj_horizon",           0.05);
    declare_parameter<std::string>("base_cmd_topic",    "/diff_drive_controller/cmd_vel");
    declare_parameter<std::string>("arm_cmd_topic",     "/jaka_arm_controller/joint_trajectory");
    declare_parameter<std::string>("odom_topic",        "/diff_drive_controller/odom");
    declare_parameter<std::string>("joint_state_topic", "/joint_states");
    declare_parameter<bool>("use_stamped_cmd",          true);   // *** new ***
    declare_parameter<std::vector<std::string>>(
        "arm_joint_names",
        std::vector<std::string>{"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"});
    declare_parameter<std::string>("base_frame",  "base_footprint");
    declare_parameter<std::string>("world_frame", "odom");
    declare_parameter<std::string>("ee_frame",    "tool0");
    // *** new ***: true  -> 初始 target = 当前全身状态 (stateDim 维), 配合 WholeBodyTrajectoryCost
    //              false -> 初始 target = 当前 EE 位姿 (7 维),      配合 EndEffectorConstraint
    declare_parameter<bool>("use_whole_body_target", true);

    taskFile_       = get_parameter("taskFile").as_string();
    libFolder_      = get_parameter("libFolder").as_string();
    urdfFile_       = get_parameter("urdfFile").as_string();
    mrtRate_        = get_parameter("mrt_loop_rate").as_double();
    trajHorizon_    = get_parameter("traj_horizon").as_double();
    armJointNames_  = get_parameter("arm_joint_names").as_string_array();
    baseFrame_      = get_parameter("base_frame").as_string();
    worldFrame_     = get_parameter("world_frame").as_string();
    eeFrame_        = get_parameter("ee_frame").as_string();
    useStampedCmd_  = get_parameter("use_stamped_cmd").as_bool();
    useWholeBodyTarget_ = get_parameter("use_whole_body_target").as_bool();

    armQ_.assign(armJointNames_.size(), 0.0);

    if (taskFile_.empty() || libFolder_.empty() || urdfFile_.empty()) {
      throw std::runtime_error(
          "taskFile / libFolder / urdfFile parameters must all be set.");
    }

    // ----- robot interface -----
    interface_ = std::make_unique<ocs2::mobile_manipulator::MobileManipulatorInterface>(
        taskFile_, libFolder_, urdfFile_);

    // ----- dims -----
    const auto& info = interface_->getManipulatorModelInfo();
    stateDim_ = info.stateDim;
    inputDim_ = info.inputDim;
    armDim_   = info.armDim;

    RCLCPP_INFO(get_logger(),
                "OCS2 model dims  state=%zu  input=%zu  arm=%zu",
                stateDim_, inputDim_, armDim_);
    if (armDim_ != armJointNames_.size()) {
      RCLCPP_FATAL(get_logger(),
                   "armDim (%zu) != arm_joint_names size (%zu). "
                   "Check task.info -> removeJoints.",
                   armDim_, armJointNames_.size());
      throw std::runtime_error("armDim mismatch");
    }

    // ----- TF -----
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // ----- pubs -----
    const std::string baseTopic = get_parameter("base_cmd_topic").as_string();
    if (useStampedCmd_) {
      base_cmd_stamped_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
          baseTopic, 10);
      RCLCPP_INFO(get_logger(),
                  "Publishing base cmd as TwistStamped on %s", baseTopic.c_str());
    } else {
      base_cmd_unstamped_pub_ = create_publisher<geometry_msgs::msg::Twist>(
          baseTopic, 10);
      RCLCPP_INFO(get_logger(),
                  "Publishing base cmd as Twist on %s (real-robot mode)",
                  baseTopic.c_str());
    }
    arm_cmd_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
        get_parameter("arm_cmd_topic").as_string(), 10);

    // ----- subs (在主 node 上, 由 main() 里的 executor spin) -----
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        get_parameter("odom_topic").as_string(),
        rclcpp::SensorDataQoS(),
        std::bind(&TracerJakaMrtBridge::odomCallback, this, std::placeholders::_1));
    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        get_parameter("joint_state_topic").as_string(),
        rclcpp::SensorDataQoS(),
        std::bind(&TracerJakaMrtBridge::jsCallback, this, std::placeholders::_1));
  }

  /// 给 MRT 一个单独的内部 node, 这样 MRT 自己的 executor 不会跟我们抢主 node.
  void initMrt() {
    ocs2InternalNode_ = std::make_shared<rclcpp::Node>(
        std::string(get_name()) + "_ocs2_internal");
    mrt_ = std::make_unique<ocs2::MRT_ROS_Interface>("mobile_manipulator");
    mrt_->initRollout(&interface_->getRollout());
    mrt_->launchNodes(ocs2InternalNode_);
  }

  /// MRT 主循环 (阻塞)
  void run() {
    RCLCPP_INFO(get_logger(), "Waiting for odom and joint_states ...");
    rclcpp::Rate wait(10);
    while (rclcpp::ok() && (!gotOdom_ || !gotJs_)) wait.sleep();
    if (!rclcpp::ok()) return;

    // ---- 初始观测 ----
    ocs2::SystemObservation initObs;
    initObs.state.setZero(stateDim_);
    initObs.input.setZero(inputDim_);
    initObs.time = 0.0;
    initObs.mode = 0;
    {
      std::lock_guard<std::mutex> lk(stateMutex_);
      fillStateLocked(initObs.state);
    }

    // ------------------------------------------------------------------
    // ---- 初始 target ----
    //
    // *** 全身轨迹模式的关键修改 ***
    //
    // 旧版: initTarget = 当前 EE 位姿 (7 维), 由 EndEffectorConstraint 解读。
    // 新版: initTarget = 当前全身状态 (stateDim 维), 由 WholeBodyTrajectoryCost 解读,
    //       语义是 "保持当前构型不动"。
    //
    // 如果这里仍然发 7 维 EE pose, 那么在真正的全身轨迹到来之前,
    // WholeBodyTrajectoryCost 会因为维度不匹配而临时失效 (退化为 0 cost),
    // 机器人在这段时间里没有任何跟踪目标。
    // ------------------------------------------------------------------
    if (useWholeBodyTarget_) {
      ocs2::TargetTrajectories initTargetTrajectories(
          {0.0}, {initObs.state}, {ocs2::vector_t::Zero(inputDim_)});
      mrt_->resetMpcNode(initTargetTrajectories);

      std::stringstream ss;
      ss << initObs.state.transpose();
      RCLCPP_INFO(get_logger(),
                  "Initial whole-body target (stay in place): [%s]",
                  ss.str().c_str());
    } else {
      // 兼容旧的 EE target 模式 (需要 task.info 里 endEffector.activate = true)
      const ocs2::vector_t initEEPose = lookupCurrentEEPose();
      RCLCPP_INFO(get_logger(),
                  "Initial EE target  pos=(%.3f, %.3f, %.3f)  quat=(%.3f, %.3f, %.3f, %.3f)",
                  initEEPose(0), initEEPose(1), initEEPose(2),
                  initEEPose(3), initEEPose(4), initEEPose(5), initEEPose(6));

      ocs2::TargetTrajectories initTargetTrajectories(
          {0.0}, {initEEPose}, {ocs2::vector_t::Zero(inputDim_)});
      mrt_->resetMpcNode(initTargetTrajectories);
    }

    RCLCPP_INFO(get_logger(), "Waiting for first MPC policy ...");
    while (rclcpp::ok() && !mrt_->initialPolicyReceived()) {
      mrt_->setCurrentObservation(initObs);
      mrt_->spinMRT();
      std::this_thread::sleep_for(50ms);
    }
    if (!rclcpp::ok()) return;
    RCLCPP_INFO(get_logger(),
                "Got first policy. Entering MRT control loop @ %.0f Hz",
                mrtRate_);

    // ---- 主循环 ----
    rclcpp::Rate rate(mrtRate_);
    const auto t0 = now();
    while (rclcpp::ok()) {
      mrt_->spinMRT();

      ocs2::SystemObservation obs;
      obs.state.setZero(stateDim_);
      obs.input.setZero(inputDim_);
      obs.time = (now() - t0).seconds();
      obs.mode = 0;
      {
        std::lock_guard<std::mutex> lk(stateMutex_);
        fillStateLocked(obs.state);
      }

      mrt_->setCurrentObservation(obs);
      mrt_->updatePolicy();

      // 当前时刻的 input 直接给底盘
      ocs2::vector_t optState, optInput;
      size_t mode;
      mrt_->evaluatePolicy(obs.time, obs.state, optState, optInput, mode);
      publishBaseCommand(optInput);

      // traj_horizon 之后的 state 给机械臂 (单点, 仅 position)
      publishArmCommand(obs.time, obs.state);

      rate.sleep();
    }
  }

  /// 显式释放 OCS2 资源 (析构顺序敏感, main 退出前调用)
  void shutdownOcs2() {
    mrt_.reset();
    ocs2InternalNode_.reset();
  }

 private:
  // -------- callbacks --------
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(stateMutex_);
    baseX_   = msg->pose.pose.position.x;
    baseY_   = msg->pose.pose.position.y;
    tf2::Quaternion q(msg->pose.pose.orientation.x,
                      msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z,
                      msg->pose.pose.orientation.w);
    double r, p, y;
    tf2::Matrix3x3(q).getRPY(r, p, y);
    baseYaw_ = y;
    gotOdom_ = true;
  }

  void jsCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(stateMutex_);
    for (size_t i = 0; i < armJointNames_.size(); ++i) {
      auto it = std::find(msg->name.begin(), msg->name.end(), armJointNames_[i]);
      if (it != msg->name.end()) {
        const size_t idx = std::distance(msg->name.begin(), it);
        if (idx < msg->position.size()) armQ_[i] = msg->position[idx];
      }
    }
    gotJs_ = true;
  }

  // -------- helpers --------
  void fillStateLocked(ocs2::vector_t& state) const {
    state.resize(stateDim_);
    state.setZero();
    state(0) = baseX_;
    state(1) = baseY_;
    state(2) = baseYaw_;
    for (size_t i = 0; i < armDim_ && i < armQ_.size(); ++i)
      state(3 + i) = armQ_[i];
  }

  /// 通过 TF 查 world_frame -> ee_frame, 返回 [pos(3); quat-xyzw(4)]
  ocs2::vector_t lookupCurrentEEPose() {
    rclcpp::Rate r(10);
    int retry = 30;
    while (rclcpp::ok() && retry-- > 0) {
      if (tf_buffer_->canTransform(worldFrame_, eeFrame_, tf2::TimePointZero,
                                    tf2::durationFromSec(0.1))) {
        break;
      }
      r.sleep();
    }

    try {
      const auto tf = tf_buffer_->lookupTransform(
          worldFrame_, eeFrame_, tf2::TimePointZero,
          tf2::durationFromSec(1.0));
      ocs2::vector_t pose(7);
      pose(0) = tf.transform.translation.x;
      pose(1) = tf.transform.translation.y;
      pose(2) = tf.transform.translation.z;
      pose(3) = tf.transform.rotation.x;
      pose(4) = tf.transform.rotation.y;
      pose(5) = tf.transform.rotation.z;
      pose(6) = tf.transform.rotation.w;
      return pose;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(get_logger(),
                  "TF lookup %s -> %s failed: %s. Using fallback pose.",
                  worldFrame_.c_str(), eeFrame_.c_str(), ex.what());
      ocs2::vector_t pose(7);
      double bx, by;
      {
        std::lock_guard<std::mutex> lk(stateMutex_);
        bx = baseX_; by = baseY_;
      }
      pose << bx + 0.5, by, 0.5, 0.0, 0.0, 0.0, 1.0;
      return pose;
    }
  }

  void publishBaseCommand(const ocs2::vector_t& input) {
    if (input.size() < 2) return;

    if (useStampedCmd_) {
      geometry_msgs::msg::TwistStamped msg;
      msg.header.stamp    = now();
      msg.header.frame_id = baseFrame_;
      msg.twist.linear.x  = input(0);
      msg.twist.angular.z = input(1);
      base_cmd_stamped_pub_->publish(msg);
    } else {
      geometry_msgs::msg::Twist msg;
      msg.linear.x  = input(0);
      msg.angular.z = input(1);
      base_cmd_unstamped_pub_->publish(msg);
    }
  }

  /// 单点 trajectory, *只* 写 position, 不写 velocity.
  void publishArmCommand(double currentTime, const ocs2::vector_t& currentState) {
    // —— 防越界：把查询时间 clamp 到 policy 末端之前一点 ——
    double queryTime = currentTime + trajHorizon_;
    const auto& policy = mrt_->getPolicy();
    if (!policy.timeTrajectory_.empty()) {
      const double tEnd = policy.timeTrajectory_.back();
      constexpr double kSafety = 1e-3;
      if (queryTime > tEnd - kSafety) {
        queryTime = std::max(currentTime, tEnd - kSafety);
      }
    }

    ocs2::vector_t optState, optInput;
    size_t mode;
    mrt_->evaluatePolicy(queryTime, currentState, optState, optInput, mode);

    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = now();
    traj.joint_names  = armJointNames_;

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.resize(armJointNames_.size());
    for (size_t i = 0; i < armJointNames_.size(); ++i)
      pt.positions[i] = optState(3 + i);

    const double dt = std::max(0.0, queryTime - currentTime);
    pt.time_from_start.sec     = static_cast<int32_t>(dt);
    pt.time_from_start.nanosec = static_cast<uint32_t>((dt - std::floor(dt)) * 1e9);

    traj.points.push_back(std::move(pt));
    arm_cmd_pub_->publish(traj);
  }

  // -------- members --------
  std::string taskFile_, libFolder_, urdfFile_;
  std::string baseFrame_, worldFrame_, eeFrame_;
  double mrtRate_{100.0}, trajHorizon_{0.05};
  bool   useStampedCmd_{true};
  bool   useWholeBodyTarget_{true};
  std::vector<std::string> armJointNames_;

  std::unique_ptr<ocs2::mobile_manipulator::MobileManipulatorInterface> interface_;
  rclcpp::Node::SharedPtr ocs2InternalNode_;
  std::unique_ptr<ocs2::MRT_ROS_Interface> mrt_;

  size_t stateDim_{0}, inputDim_{0}, armDim_{0};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr base_cmd_stamped_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr        base_cmd_unstamped_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;

  std::mutex stateMutex_;
  std::atomic<bool> gotOdom_{false}, gotJs_{false};
  double baseX_{0.0}, baseY_{0.0}, baseYaw_{0.0};
  std::vector<double> armQ_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto bridge = std::make_shared<TracerJakaMrtBridge>();

  try {
    bridge->initMrt();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(bridge->get_logger(), "initMrt failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(bridge);
  std::thread spinner([&exec]() {
    try { exec.spin(); } catch (...) {}
  });

  try {
    bridge->run();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(bridge->get_logger(), "MRT loop exception: %s", e.what());
  }

  exec.cancel();
  if (spinner.joinable()) spinner.join();
  bridge->shutdownOcs2();
  rclcpp::shutdown();
  return 0;
}

