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
//
//  [可视化新增]
//   * 集成 TracerJakaVisualization: 每个控制周期(降频)把 MPC 预测的 EE/底盘轨迹、
//     参考全身轨迹、以及自碰撞距离发布成 RViz marker。开关参数 enable_visualization。
//   * 它【不】发布 joint_states / world->base TF (那些来自仿真或 robot_state_publisher),
//     只发轨迹 marker, 因此和你现有 TF 树不冲突。
// =============================================================================

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
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

// [可视化新增]
#include "TracerJakaVisualization.h"

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

    // ★ 安全闸: 单步臂关节最大允许变化 [rad]
    //   MPC 无解时可能输出数值爆炸, 超过此阈值即触发安全停车
    declare_parameter<double>("arm_max_delta_per_step", 0.50);

    // [可视化新增] 参数
    declare_parameter<bool>("enable_visualization", true);
    declare_parameter<bool>("viz_self_collision",   true);
    declare_parameter<double>("viz_rate",           20.0);   // marker 发布频率 [Hz]

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
    armMaxDeltaPerStep_ = get_parameter("arm_max_delta_per_step").as_double();

    enableViz_       = get_parameter("enable_visualization").as_bool();
    vizSelfCollision_= get_parameter("viz_self_collision").as_bool();
    vizRate_         = get_parameter("viz_rate").as_double();

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

    // [可视化新增] 需要 shared_from_this(), 只能在 make_shared 之后调用,
    // 因此放在 initMrt() 里 (由 main 在构造完成后调用)。
    if (enableViz_) {
      viz_ = std::make_unique<tracer_jaka::TracerJakaVisualization>(
          shared_from_this(), *interface_, worldFrame_, vizSelfCollision_);
      vizEveryN_ = std::max(1, static_cast<int>(std::round(
          mrtRate_ / std::max(1.0, vizRate_))));
      RCLCPP_INFO(get_logger(),
                  "Visualization enabled: 每 %d 个控制周期发布一次 marker (~%.1f Hz).",
                  vizEveryN_, mrtRate_ / vizEveryN_);
    }
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

    // ---- 安全闸: plan 过期计数器 ----
    int planExpiredCount = 0;
    static constexpr int kPlanExpiredMaxLog = 5;

    // ---- 计时/频率统计 ----
    using clk = std::chrono::steady_clock;
    lastReport_ = now();
    while (rclcpp::ok()) {
      const auto workBegin = clk::now();   // 一次控制迭代的"计算"起点

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
      const bool policyUpdated = mrt_->updatePolicy();   // 是否拿到"新"策略
      if (policyUpdated) ++policyUpdateCount_;

      // ==================================================================
      // ★ 安全闸: 检查 plan 是否过期
      //   如果 currentTime > plan_end, MPC 没及时更新策略,
      //   evaluatePolicy 会用到过期数据, 输出不可预测的控制量。
      //   直接零指令, 跳过策略评估, 等待 MPC 恢复。
      // ==================================================================
      bool planValid = false;
      try {
        const auto& policy = mrt_->getPolicy();
        if (!policy.timeTrajectory_.empty()) {
          planValid = (obs.time <= policy.timeTrajectory_.back());
        }
      } catch (...) { planValid = false; }

      if (planValid) {
        planExpiredCount = 0;

        // 当前时刻的 input 直接给底盘
        ocs2::vector_t optState, optInput;
        size_t mode;
        mrt_->evaluatePolicy(obs.time, obs.state, optState, optInput, mode);

        // ================================================================
        // ★ 内容安全闸: 检查 MPC 输出的臂状态是否合理
        //   MPC 在极限位姿无解时可能产生数值爆炸 (时间合法但内容非法),
        //   此时 obs.time ≤ plan_end 所以时间闸不触发, 但臂会飞。
        //   如果任意关节相对当前观测值跳变超过阈值 → 触发安全停车。
        // ================================================================
        bool contentSafe = true;
        {
          std::lock_guard<std::mutex> lk(stateMutex_);
          for (size_t i = 0; i < armDim_; ++i) {
            const double curQ = (i < armQ_.size()) ? armQ_[i] : 0.0;
            const double delta = std::abs(optState(3 + i) - curQ);
            if (delta > armMaxDeltaPerStep_) {
              RCLCPP_ERROR(get_logger(),
                           "[SAFETY] Arm content check FAILED! "
                           "joint_%zu: mpc=%.3f obs=%.3f delta=%.3f > limit=%.3f",
                           i + 1, optState(3 + i), curQ, delta, armMaxDeltaPerStep_);
              contentSafe = false;
              break;
            }
          }
        }

        if (contentSafe) {
          publishBaseCommand(optInput);

          // traj_horizon 之后的 state 给机械臂 (单点, 仅 position)
          publishArmCommand(obs.time, obs.state);

          // ★ 保存最后一次有效 MPC 输出的臂状态, 供安全闸 hold 时使用
          lastGoodArmQ_.resize(armDim_);
          for (size_t i = 0; i < armDim_; ++i)
            lastGoodArmQ_[i] = optState(3 + i);
        } else {
          // 内容不安全 → 触发零指令 (和 plan 过期一样处理)
          RCLCPP_ERROR(get_logger(),
                       "[SAFETY] MPC arm output unsafe! Zeroing all control.");
          publishZeroBaseCommand();
          publishHoldArmCommand();
        }
      } else {
        // ★ 时间安全闸: plan 过期 → 零指令
        ++planExpiredCount;
        if (planExpiredCount <= kPlanExpiredMaxLog) {
          RCLCPP_ERROR(get_logger(),
                       "[SAFETY] Plan time expired! currentTime=%.3f > planEnd=%.3f. "
                       "Zeroing all control. (count=%d)",
                       obs.time,
                       mrt_->getPolicy().timeTrajectory_.back(),
                       planExpiredCount);
        } else if (planExpiredCount == kPlanExpiredMaxLog + 1) {
          RCLCPP_ERROR(get_logger(),
                       "[SAFETY] Plan still expired, suppressing further logs...");
        }

        publishZeroBaseCommand();
        publishHoldArmCommand();
      }

      // [可视化新增] 降频发布 marker
      if (viz_ && (++vizCounter_ % vizEveryN_ == 0)) {
        try {
          viz_->update(obs, mrt_->getPolicy(), mrt_->getCommand());
        } catch (const std::exception& e) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                               "visualization update failed: %s", e.what());
        }
      }

      // —— 计算耗时 (不含 rate.sleep 的等待) ——
      const double workMs =
          std::chrono::duration<double, std::milli>(clk::now() - workBegin).count();
      loopWorkSumMs_ += workMs;
      loopWorkMaxMs_ = std::max(loopWorkMaxMs_, workMs);
      ++loopCount_;

      // —— "计划新鲜度" = 当前观测时间 - 生成当前策略所用观测的时间 ——
      // 近似反映 观测->求解->回传 的端到端控制延迟。
      const double planAgeMs =
          1000.0 * (obs.time - mrt_->getCommand().mpcInitObservation_.time);
      planAgeSumMs_ += planAgeMs;
      planAgeMaxMs_ = std::max(planAgeMaxMs_, planAgeMs);

      // —— 每 ~2s 汇报一次 ——
      const double reportDt = (now() - lastReport_).seconds();
      if (reportDt >= 2.0 && loopCount_ > 0) {
        const double ctrlHz     = loopCount_ / reportDt;
        const double mpcSeenHz  = policyUpdateCount_ / reportDt;   // MRT 侧看到的新策略频率
        const double avgWorkMs  = loopWorkSumMs_ / loopCount_;
        const double avgPlanMs  = planAgeSumMs_ / loopCount_;
        RCLCPP_INFO(get_logger(),
                    "[timing] ctrl_loop=%.1f Hz (target %.0f) | work/iter avg=%.2f ms max=%.2f ms | "
                    "MPC_policy_seen=%.1f Hz | plan_age avg=%.1f ms max=%.1f ms",
                    ctrlHz, mrtRate_, avgWorkMs, loopWorkMaxMs_,
                    mpcSeenHz, avgPlanMs, planAgeMaxMs_);

        // 复位窗口
        lastReport_        = now();
        loopCount_         = 0;
        policyUpdateCount_ = 0;
        loopWorkSumMs_     = 0.0;
        loopWorkMaxMs_     = 0.0;
        planAgeSumMs_      = 0.0;
        planAgeMaxMs_      = 0.0;
      }

      rate.sleep();
    }
  }

  /// 显式释放 OCS2 资源 (析构顺序敏感, main 退出前调用)
  void shutdownOcs2() {
    viz_.reset();            // [可视化新增] 先于 mrt_/interface_ 释放
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

  /// ★ 安全闸: 底盘零速度
  void publishZeroBaseCommand() {
    if (useStampedCmd_) {
      geometry_msgs::msg::TwistStamped msg;
      msg.header.stamp    = now();
      msg.header.frame_id = baseFrame_;
      msg.twist.linear.x  = 0.0;
      msg.twist.angular.z = 0.0;
      base_cmd_stamped_pub_->publish(msg);
    } else {
      geometry_msgs::msg::Twist msg;
      msg.linear.x  = 0.0;
      msg.angular.z = 0.0;
      base_cmd_unstamped_pub_->publish(msg);
    }
  }

  /// ★ 安全闸: 机械臂保持上次有效 MPC 输出的位置 (对抗重力, 不会下坠)
  void publishHoldArmCommand() {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = now();
    traj.joint_names  = armJointNames_;

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.resize(armJointNames_.size());
    if (!lastGoodArmQ_.empty()) {
      // 用上次 MPC 输出的有效臂状态, 而不是当前已下坠的关节角
      for (size_t i = 0; i < armJointNames_.size() && i < lastGoodArmQ_.size(); ++i)
        pt.positions[i] = lastGoodArmQ_[i];
    } else {
      // 冷启动: 还没有收到过有效 MPC 策略, 回退到 home
      static const std::vector<double> kArmHome =
          {-0.515, 1.5707, -1.5707, 1.5707, 1.5707, 0.254};
      for (size_t i = 0; i < armJointNames_.size() && i < kArmHome.size(); ++i)
        pt.positions[i] = kArmHome[i];
    }
    pt.time_from_start.sec  = 0;
    pt.time_from_start.nanosec = 50000000;  // 50ms
    traj.points.push_back(std::move(pt));
    arm_cmd_pub_->publish(traj);
  }

  // -------- members --------
  std::string taskFile_, libFolder_, urdfFile_;
  std::string baseFrame_, worldFrame_, eeFrame_;
  double mrtRate_{100.0}, trajHorizon_{0.05};
  bool   useStampedCmd_{true};
  bool   useWholeBodyTarget_{true};
  double armMaxDeltaPerStep_{0.50};   // ★ 臂关节单步最大跳变 [rad]
  std::vector<std::string> armJointNames_;

  // [可视化新增]
  bool   enableViz_{true};
  bool   vizSelfCollision_{true};
  double vizRate_{20.0};
  int    vizEveryN_{5};
  long   vizCounter_{0};
  std::unique_ptr<tracer_jaka::TracerJakaVisualization> viz_;

  // [计时统计] 每 ~2s 汇报一次控制环频率 / 计算耗时 / 计划新鲜度
  rclcpp::Time lastReport_;
  size_t loopCount_{0};
  size_t policyUpdateCount_{0};
  double loopWorkSumMs_{0.0};
  double loopWorkMaxMs_{0.0};
  double planAgeSumMs_{0.0};
  double planAgeMaxMs_{0.0};

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
  std::vector<double> lastGoodArmQ_;   // ★ 安全闸: 最后一次有效 MPC 输出的臂位姿
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