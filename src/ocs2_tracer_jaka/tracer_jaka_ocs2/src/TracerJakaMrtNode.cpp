// =============================================================================
//  TracerJakaMrtNode.cpp
//
//  OCS2 MRT 节点 + 控制器桥（ROS 2）
//
//  底盘：
//    use_stamped_cmd=true  -> geometry_msgs/TwistStamped
//    use_stamped_cmd=false -> geometry_msgs/Twist
//
//  机械臂：
//    forward_command_controller/ForwardCommandController
//    发布 std_msgs/Float64MultiArray 到：
//      /jaka_forward_controller/commands
//
//    数组顺序必须与控制器 YAML 中 joints 顺序一致。
// =============================================================================

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mobile_manipulator/ManipulatorModelInfo.h>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include "TracerJakaVisualization.h"

using namespace std::chrono_literals;

class TracerJakaMrtBridge : public rclcpp::Node
{
public:
  TracerJakaMrtBridge()
      : Node("tracer_jaka_mrt_node")
  {
    // -------------------------------------------------------------------------
    // 参数
    // -------------------------------------------------------------------------
    declare_parameter<std::string>("taskFile", "");
    declare_parameter<std::string>("libFolder", "");
    declare_parameter<std::string>("urdfFile", "");

    declare_parameter<double>("mrt_loop_rate", 100.0);
    declare_parameter<double>("traj_horizon", 0.05);

    declare_parameter<std::string>(
        "base_cmd_topic", "/diff_drive_controller/cmd_vel");

    // ForwardCommandController 的命令话题
    declare_parameter<std::string>(
        "arm_cmd_topic", "/jaka_forward_controller/commands");

    declare_parameter<std::string>(
        "odom_topic", "/diff_drive_controller/odom");
    declare_parameter<std::string>(
        "joint_state_topic", "/joint_states");

    declare_parameter<bool>("use_stamped_cmd", true);

    declare_parameter<std::vector<std::string>>(
        "arm_joint_names",
        std::vector<std::string>{
            "joint_1", "joint_2", "joint_3",
            "joint_4", "joint_5", "joint_6"});

    declare_parameter<std::string>("base_frame", "base_footprint");
    declare_parameter<std::string>("world_frame", "odom");
    declare_parameter<std::string>("ee_frame", "tool0");

    declare_parameter<bool>("use_whole_body_target", true);

    // 每次命令相对当前实测位置允许的最大变化
    declare_parameter<double>("arm_max_delta_per_step", 0.50);

    // 可视化
    declare_parameter<bool>("enable_visualization", true);
    declare_parameter<bool>("viz_self_collision", true);
    declare_parameter<double>("viz_rate", 20.0);

    taskFile_ = get_parameter("taskFile").as_string();
    libFolder_ = get_parameter("libFolder").as_string();
    urdfFile_ = get_parameter("urdfFile").as_string();

    mrtRate_ = get_parameter("mrt_loop_rate").as_double();
    trajHorizon_ = get_parameter("traj_horizon").as_double();

    armJointNames_ =
        get_parameter("arm_joint_names").as_string_array();

    baseFrame_ = get_parameter("base_frame").as_string();
    worldFrame_ = get_parameter("world_frame").as_string();
    eeFrame_ = get_parameter("ee_frame").as_string();

    useStampedCmd_ =
        get_parameter("use_stamped_cmd").as_bool();

    useWholeBodyTarget_ =
        get_parameter("use_whole_body_target").as_bool();

    armMaxDeltaPerStep_ =
        get_parameter("arm_max_delta_per_step").as_double();

    enableViz_ =
        get_parameter("enable_visualization").as_bool();

    vizSelfCollision_ =
        get_parameter("viz_self_collision").as_bool();

    vizRate_ =
        get_parameter("viz_rate").as_double();

    armQ_.assign(armJointNames_.size(), 0.0);

    if (taskFile_.empty() ||
        libFolder_.empty() ||
        urdfFile_.empty())
    {
      throw std::runtime_error(
          "taskFile / libFolder / urdfFile parameters must all be set.");
    }

    if (mrtRate_ <= 0.0)
    {
      throw std::runtime_error("mrt_loop_rate must be positive.");
    }

    if (trajHorizon_ < 0.0)
    {
      throw std::runtime_error("traj_horizon must not be negative.");
    }

    if (armMaxDeltaPerStep_ <= 0.0)
    {
      throw std::runtime_error(
          "arm_max_delta_per_step must be positive.");
    }

    // -------------------------------------------------------------------------
    // OCS2 robot interface
    // -------------------------------------------------------------------------
    interface_ = std::make_unique<
        ocs2::mobile_manipulator::MobileManipulatorInterface>(
        taskFile_, libFolder_, urdfFile_);

    const auto &info = interface_->getManipulatorModelInfo();

    stateDim_ = info.stateDim;
    inputDim_ = info.inputDim;
    armDim_ = info.armDim;

    RCLCPP_INFO(
        get_logger(),
        "OCS2 model dims: state=%zu input=%zu arm=%zu",
        stateDim_, inputDim_, armDim_);

    if (stateDim_ < 3 + armDim_)
    {
      RCLCPP_FATAL(
          get_logger(),
          "Invalid OCS2 state dimension: stateDim=%zu, required >= %zu",
          stateDim_, 3 + armDim_);
      throw std::runtime_error("Invalid OCS2 state dimension.");
    }

    if (inputDim_ < 2)
    {
      RCLCPP_FATAL(
          get_logger(),
          "Invalid OCS2 input dimension: inputDim=%zu, required >= 2",
          inputDim_);
      throw std::runtime_error("Invalid OCS2 input dimension.");
    }

    if (armDim_ != armJointNames_.size())
    {
      RCLCPP_FATAL(
          get_logger(),
          "armDim (%zu) != arm_joint_names size (%zu). "
          "Check task.info -> removeJoints.",
          armDim_, armJointNames_.size());

      throw std::runtime_error("armDim mismatch.");
    }

    // -------------------------------------------------------------------------
    // TF
    // -------------------------------------------------------------------------
    tf_buffer_ =
        std::make_unique<tf2_ros::Buffer>(get_clock());

    tf_listener_ =
        std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // -------------------------------------------------------------------------
    // Publishers
    // -------------------------------------------------------------------------
    const std::string baseTopic =
        get_parameter("base_cmd_topic").as_string();

    if (useStampedCmd_)
    {
      base_cmd_stamped_pub_ =
          create_publisher<geometry_msgs::msg::TwistStamped>(
              baseTopic, 10);

      RCLCPP_INFO(
          get_logger(),
          "Publishing base commands as TwistStamped on %s",
          baseTopic.c_str());
    }
    else
    {
      base_cmd_unstamped_pub_ =
          create_publisher<geometry_msgs::msg::Twist>(
              baseTopic, 10);

      RCLCPP_INFO(
          get_logger(),
          "Publishing base commands as Twist on %s",
          baseTopic.c_str());
    }

    const std::string armTopic =
        get_parameter("arm_cmd_topic").as_string();

    // ForwardCommandController 使用 Float64MultiArray
    arm_cmd_pub_ =
        create_publisher<std_msgs::msg::Float64MultiArray>(
            armTopic,
            rclcpp::QoS(10).reliable());

    RCLCPP_INFO(
        get_logger(),
        "Publishing arm position commands as "
        "std_msgs/msg/Float64MultiArray on %s",
        armTopic.c_str());

    std::stringstream jointStream;
    for (size_t i = 0; i < armJointNames_.size(); ++i)
    {
      if (i != 0)
      {
        jointStream << ", ";
      }
      jointStream << armJointNames_[i];
    }

    RCLCPP_INFO(
        get_logger(),
        "Forward controller joint command order: [%s]",
        jointStream.str().c_str());

    // -------------------------------------------------------------------------
    // Subscribers
    // -------------------------------------------------------------------------
    odom_sub_ =
        create_subscription<nav_msgs::msg::Odometry>(
            get_parameter("odom_topic").as_string(),
            rclcpp::SensorDataQoS(),
            std::bind(
                &TracerJakaMrtBridge::odomCallback,
                this,
                std::placeholders::_1));

    js_sub_ =
        create_subscription<sensor_msgs::msg::JointState>(
            get_parameter("joint_state_topic").as_string(),
            rclcpp::SensorDataQoS(),
            std::bind(
                &TracerJakaMrtBridge::jsCallback,
                this,
                std::placeholders::_1));
  }

  void initMrt()
  {
    ocs2InternalNode_ =
        std::make_shared<rclcpp::Node>(
            std::string(get_name()) + "_ocs2_internal");

    mrt_ =
        std::make_unique<ocs2::MRT_ROS_Interface>(
            "mobile_manipulator");

    mrt_->initRollout(&interface_->getRollout());
    mrt_->launchNodes(ocs2InternalNode_);

    if (enableViz_)
    {
      viz_ =
          std::make_unique<tracer_jaka::TracerJakaVisualization>(
              shared_from_this(),
              *interface_,
              worldFrame_,
              vizSelfCollision_);

      vizEveryN_ = std::max(
          1,
          static_cast<int>(
              std::round(
                  mrtRate_ / std::max(1.0, vizRate_))));

      RCLCPP_INFO(
          get_logger(),
          "Visualization enabled: publishing every %d control cycles "
          "(approximately %.1f Hz)",
          vizEveryN_,
          mrtRate_ / static_cast<double>(vizEveryN_));
    }
  }

  void run()
  {
    RCLCPP_INFO(
        get_logger(),
        "Waiting for odom and complete joint_states...");

    rclcpp::Rate waitRate(10.0);

    while (rclcpp::ok() &&
           (!gotOdom_.load() || !gotJs_.load()))
    {
      waitRate.sleep();
    }

    if (!rclcpp::ok())
    {
      return;
    }

    // -------------------------------------------------------------------------
    // 初始观测
    // -------------------------------------------------------------------------
    ocs2::SystemObservation initObs;
    initObs.state.setZero(stateDim_);
    initObs.input.setZero(inputDim_);
    initObs.time = 0.0;
    initObs.mode = 0;

    {
      std::lock_guard<std::mutex> lock(stateMutex_);
      fillStateLocked(initObs.state);
    }

    // 冷启动保持位置也使用当前实测关节角，不再使用固定 home。
    {
      std::lock_guard<std::mutex> lock(stateMutex_);
      lastGoodArmQ_ = armQ_;
    }

    // -------------------------------------------------------------------------
    // 初始目标
    // -------------------------------------------------------------------------
    if (useWholeBodyTarget_)
    {
      ocs2::TargetTrajectories initTargetTrajectories(
          {0.0},
          {initObs.state},
          {ocs2::vector_t::Zero(inputDim_)});

      mrt_->resetMpcNode(initTargetTrajectories);

      std::stringstream stateStream;
      stateStream << initObs.state.transpose();

      RCLCPP_INFO(
          get_logger(),
          "Initial whole-body target: [%s]",
          stateStream.str().c_str());
    }
    else
    {
      const ocs2::vector_t initEEPose =
          lookupCurrentEEPose();

      RCLCPP_INFO(
          get_logger(),
          "Initial EE target: "
          "pos=(%.3f, %.3f, %.3f), "
          "quat=(%.3f, %.3f, %.3f, %.3f)",
          initEEPose(0),
          initEEPose(1),
          initEEPose(2),
          initEEPose(3),
          initEEPose(4),
          initEEPose(5),
          initEEPose(6));

      ocs2::TargetTrajectories initTargetTrajectories(
          {0.0},
          {initEEPose},
          {ocs2::vector_t::Zero(inputDim_)});

      mrt_->resetMpcNode(initTargetTrajectories);
    }

    // -------------------------------------------------------------------------
    // 等待第一条 MPC policy
    // -------------------------------------------------------------------------
    RCLCPP_INFO(
        get_logger(),
        "Waiting for first MPC policy...");

    while (rclcpp::ok() &&
           !mrt_->initialPolicyReceived())
    {
      mrt_->setCurrentObservation(initObs);
      mrt_->spinMRT();
      std::this_thread::sleep_for(50ms);
    }

    if (!rclcpp::ok())
    {
      return;
    }

    RCLCPP_INFO(
        get_logger(),
        "Got first MPC policy. Entering MRT loop at %.1f Hz",
        mrtRate_);

    // -------------------------------------------------------------------------
    // MRT 控制循环
    // -------------------------------------------------------------------------
    rclcpp::Rate rate(mrtRate_);
    const rclcpp::Time t0 = now();

    int planExpiredCount = 0;
    static constexpr int kPlanExpiredMaxLog = 5;

    using SteadyClock = std::chrono::steady_clock;

    lastReport_ = now();

    while (rclcpp::ok())
    {
      const auto workBegin = SteadyClock::now();

      mrt_->spinMRT();

      ocs2::SystemObservation obs;
      obs.state.setZero(stateDim_);
      obs.input.setZero(inputDim_);
      obs.time = (now() - t0).seconds();
      obs.mode = 0;

      {
        std::lock_guard<std::mutex> lock(stateMutex_);
        fillStateLocked(obs.state);
      }

      mrt_->setCurrentObservation(obs);

      const bool policyUpdated = mrt_->updatePolicy();
      if (policyUpdated)
      {
        ++policyUpdateCount_;
      }

      // -----------------------------------------------------------------------
      // 检查 policy 时间范围
      // -----------------------------------------------------------------------
      bool planValid = false;
      double planEnd = std::numeric_limits<double>::quiet_NaN();

      try
      {
        const auto &policy = mrt_->getPolicy();

        if (!policy.timeTrajectory_.empty())
        {
          planEnd = policy.timeTrajectory_.back();

          planValid =
              std::isfinite(planEnd) &&
              obs.time <= planEnd;
        }
      }
      catch (const std::exception &e)
      {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "Failed to inspect MPC policy: %s",
            e.what());

        planValid = false;
      }
      catch (...)
      {
        planValid = false;
      }

      if (planValid)
      {
        planExpiredCount = 0;

        ocs2::vector_t currentOptState;
        ocs2::vector_t currentOptInput;
        size_t currentMode = 0;

        bool policyEvaluationSucceeded = true;

        try
        {
          mrt_->evaluatePolicy(
              obs.time,
              obs.state,
              currentOptState,
              currentOptInput,
              currentMode);
        }
        catch (const std::exception &e)
        {
          RCLCPP_ERROR(
              get_logger(),
              "[SAFETY] Current policy evaluation failed: %s",
              e.what());

          policyEvaluationSucceeded = false;
        }
        catch (...)
        {
          RCLCPP_ERROR(
              get_logger(),
              "[SAFETY] Current policy evaluation failed "
              "with an unknown exception.");

          policyEvaluationSucceeded = false;
        }

        if (!policyEvaluationSucceeded ||
            !isPolicyVectorValid(
                currentOptState,
                currentOptInput))
        {
          RCLCPP_ERROR_THROTTLE(
              get_logger(),
              *get_clock(),
              1000,
              "[SAFETY] MPC current output is invalid. "
              "Stopping base and holding arm.");

          publishZeroBaseCommand();
          publishHoldArmCommand();
        }
        else
        {
          // 底盘使用当前时刻 MPC 输入
          const bool baseSafe =
              std::isfinite(currentOptInput(0)) &&
              std::isfinite(currentOptInput(1));

          if (!baseSafe)
          {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "[SAFETY] MPC base command contains NaN/Inf.");

            publishZeroBaseCommand();
            publishHoldArmCommand();
          }
          else
          {
            // 机械臂仍然使用 traj_horizon 后的预测状态，
            // 但输出格式改为 Float64MultiArray。
            std::vector<double> armCommand;

            const bool armCommandSafe =
                computeSafeArmCommand(
                    obs.time,
                    obs.state,
                    armCommand);

            if (armCommandSafe)
            {
              publishBaseCommand(currentOptInput);
              publishArmPositions(armCommand);

              // 保存真正发送给 forward controller 的命令
              lastGoodArmQ_ = armCommand;
            }
            else
            {
              RCLCPP_ERROR_THROTTLE(
                  get_logger(),
                  *get_clock(),
                  1000,
                  "[SAFETY] Predicted arm command is unsafe. "
                  "Stopping base and holding arm.");

              publishZeroBaseCommand();
              publishHoldArmCommand();
            }
          }
        }
      }
      else
      {
        ++planExpiredCount;

        if (planExpiredCount <= kPlanExpiredMaxLog)
        {
          if (std::isfinite(planEnd))
          {
            RCLCPP_ERROR(
                get_logger(),
                "[SAFETY] MPC plan expired: "
                "currentTime=%.3f, planEnd=%.3f, count=%d. "
                "Stopping base and holding arm.",
                obs.time,
                planEnd,
                planExpiredCount);
          }
          else
          {
            RCLCPP_ERROR(
                get_logger(),
                "[SAFETY] MPC policy is empty or invalid, count=%d. "
                "Stopping base and holding arm.",
                planExpiredCount);
          }
        }
        else if (
            planExpiredCount ==
            kPlanExpiredMaxLog + 1)
        {
          RCLCPP_ERROR(
              get_logger(),
              "[SAFETY] MPC plan remains invalid; "
              "suppressing repeated logs.");
        }

        publishZeroBaseCommand();
        publishHoldArmCommand();
      }

      // -----------------------------------------------------------------------
      // 可视化
      // -----------------------------------------------------------------------
      if (viz_ &&
          (++vizCounter_ % vizEveryN_ == 0))
      {
        try
        {
          viz_->update(
              obs,
              mrt_->getPolicy(),
              mrt_->getCommand());
        }
        catch (const std::exception &e)
        {
          RCLCPP_WARN_THROTTLE(
              get_logger(),
              *get_clock(),
              5000,
              "Visualization update failed: %s",
              e.what());
        }
      }

      // -----------------------------------------------------------------------
      // 计时统计
      // -----------------------------------------------------------------------
      const double workMs =
          std::chrono::duration<double, std::milli>(
              SteadyClock::now() - workBegin)
              .count();

      loopWorkSumMs_ += workMs;
      loopWorkMaxMs_ =
          std::max(loopWorkMaxMs_, workMs);
      ++loopCount_;

      try
      {
        const double planAgeMs =
            1000.0 *
            (obs.time -
             mrt_->getCommand().mpcInitObservation_.time);

        if (std::isfinite(planAgeMs))
        {
          planAgeSumMs_ += planAgeMs;
          planAgeMaxMs_ =
              std::max(planAgeMaxMs_, planAgeMs);
        }
      }
      catch (...)
      {
        // 统计信息失败不影响控制。
      }

      const double reportDt =
          (now() - lastReport_).seconds();

      if (reportDt >= 2.0 &&
          loopCount_ > 0)
      {
        const double ctrlHz =
            static_cast<double>(loopCount_) /
            reportDt;

        const double mpcSeenHz =
            static_cast<double>(policyUpdateCount_) /
            reportDt;

        const double avgWorkMs =
            loopWorkSumMs_ /
            static_cast<double>(loopCount_);

        const double avgPlanMs =
            planAgeSumMs_ /
            static_cast<double>(loopCount_);

        RCLCPP_INFO(
            get_logger(),
            "[timing] ctrl_loop=%.1f Hz (target %.1f) | "
            "work avg=%.2f ms max=%.2f ms | "
            "MPC_policy_seen=%.1f Hz | "
            "plan_age avg=%.1f ms max=%.1f ms",
            ctrlHz,
            mrtRate_,
            avgWorkMs,
            loopWorkMaxMs_,
            mpcSeenHz,
            avgPlanMs,
            planAgeMaxMs_);

        lastReport_ = now();
        loopCount_ = 0;
        policyUpdateCount_ = 0;
        loopWorkSumMs_ = 0.0;
        loopWorkMaxMs_ = 0.0;
        planAgeSumMs_ = 0.0;
        planAgeMaxMs_ = 0.0;
      }

      rate.sleep();
    }
  }

  void shutdownOcs2()
  {
    viz_.reset();
    mrt_.reset();
    ocs2InternalNode_.reset();
  }

private:
  // ---------------------------------------------------------------------------
  // Callbacks
  // ---------------------------------------------------------------------------
  void odomCallback(
      const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(stateMutex_);

    baseX_ = msg->pose.pose.position.x;
    baseY_ = msg->pose.pose.position.y;

    tf2::Quaternion quaternion(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    baseYaw_ = yaw;
    gotOdom_.store(true);
  }

  void jsCallback(
      const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(stateMutex_);

    bool allArmJointsFound = true;

    for (size_t i = 0;
         i < armJointNames_.size();
         ++i)
    {
      const auto it =
          std::find(
              msg->name.begin(),
              msg->name.end(),
              armJointNames_[i]);

      if (it == msg->name.end())
      {
        allArmJointsFound = false;
        continue;
      }

      const size_t index =
          static_cast<size_t>(
              std::distance(
                  msg->name.begin(),
                  it));

      if (index >= msg->position.size() ||
          !std::isfinite(msg->position[index]))
      {
        allArmJointsFound = false;
        continue;
      }

      armQ_[i] = msg->position[index];
    }

    // 只有收到全部机械臂关节后才允许启动 MPC。
    if (allArmJointsFound)
    {
      gotJs_.store(true);
    }
  }

  // ---------------------------------------------------------------------------
  // 状态构造
  // ---------------------------------------------------------------------------
  void fillStateLocked(
      ocs2::vector_t &state) const
  {
    state.resize(stateDim_);
    state.setZero();

    state(0) = baseX_;
    state(1) = baseY_;
    state(2) = baseYaw_;

    for (size_t i = 0;
         i < armDim_ && i < armQ_.size();
         ++i)
    {
      state(3 + i) = armQ_[i];
    }
  }

  // ---------------------------------------------------------------------------
  // 当前末端位姿
  // ---------------------------------------------------------------------------
  ocs2::vector_t lookupCurrentEEPose()
  {
    rclcpp::Rate retryRate(10.0);
    int retry = 30;

    while (rclcpp::ok() &&
           retry-- > 0)
    {
      if (tf_buffer_->canTransform(
              worldFrame_,
              eeFrame_,
              tf2::TimePointZero,
              tf2::durationFromSec(0.1)))
      {
        break;
      }

      retryRate.sleep();
    }

    try
    {
      const auto transform =
          tf_buffer_->lookupTransform(
              worldFrame_,
              eeFrame_,
              tf2::TimePointZero,
              tf2::durationFromSec(1.0));

      ocs2::vector_t pose(7);

      pose(0) = transform.transform.translation.x;
      pose(1) = transform.transform.translation.y;
      pose(2) = transform.transform.translation.z;

      pose(3) = transform.transform.rotation.x;
      pose(4) = transform.transform.rotation.y;
      pose(5) = transform.transform.rotation.z;
      pose(6) = transform.transform.rotation.w;

      return pose;
    }
    catch (const tf2::TransformException &e)
    {
      RCLCPP_WARN(
          get_logger(),
          "TF lookup %s -> %s failed: %s. "
          "Using fallback EE pose.",
          worldFrame_.c_str(),
          eeFrame_.c_str(),
          e.what());

      double baseX = 0.0;
      double baseY = 0.0;

      {
        std::lock_guard<std::mutex> lock(stateMutex_);
        baseX = baseX_;
        baseY = baseY_;
      }

      ocs2::vector_t pose(7);
      pose << baseX + 0.5,
          baseY,
          0.5,
          0.0,
          0.0,
          0.0,
          1.0;

      return pose;
    }
  }

  // ---------------------------------------------------------------------------
  // 验证当前 policy 输出
  // ---------------------------------------------------------------------------
  bool isPolicyVectorValid(
      const ocs2::vector_t &state,
      const ocs2::vector_t &input) const
  {
    if (state.size() <
            static_cast<Eigen::Index>(
                3 + armDim_) ||
        input.size() < 2)
    {
      return false;
    }

    for (Eigen::Index i = 0;
         i < state.size();
         ++i)
    {
      if (!std::isfinite(state(i)))
      {
        return false;
      }
    }

    for (Eigen::Index i = 0;
         i < input.size();
         ++i)
    {
      if (!std::isfinite(input(i)))
      {
        return false;
      }
    }

    return true;
  }

  // ---------------------------------------------------------------------------
  // 计算 traj_horizon 后的机械臂位置命令
  // ---------------------------------------------------------------------------
  bool computeSafeArmCommand(
      double currentTime,
      const ocs2::vector_t &currentState,
      std::vector<double> &armCommand)
  {
    armCommand.clear();

    double queryTime =
        currentTime + trajHorizon_;

    try
    {
      const auto &policy = mrt_->getPolicy();

      if (policy.timeTrajectory_.empty())
      {
        return false;
      }

      const double planEnd =
          policy.timeTrajectory_.back();

      if (!std::isfinite(planEnd) ||
          currentTime > planEnd)
      {
        return false;
      }

      constexpr double kEndSafety = 1e-3;

      const double latestQueryTime =
          std::max(
              currentTime,
              planEnd - kEndSafety);

      queryTime =
          std::min(
              queryTime,
              latestQueryTime);

      ocs2::vector_t predictedState;
      ocs2::vector_t predictedInput;
      size_t predictedMode = 0;

      mrt_->evaluatePolicy(
          queryTime,
          currentState,
          predictedState,
          predictedInput,
          predictedMode);

      if (predictedState.size() <
          static_cast<Eigen::Index>(
              3 + armDim_))
      {
        RCLCPP_ERROR(
            get_logger(),
            "[SAFETY] Predicted state dimension is %ld, expected >= %zu",
            static_cast<long>(predictedState.size()),
            3 + armDim_);

        return false;
      }

      std::vector<double> currentArmQ;

      {
        std::lock_guard<std::mutex> lock(stateMutex_);
        currentArmQ = armQ_;
      }

      if (currentArmQ.size() != armDim_)
      {
        return false;
      }

      armCommand.resize(armDim_);

      for (size_t i = 0;
           i < armDim_;
           ++i)
      {
        const double command =
            predictedState(
                static_cast<Eigen::Index>(
                    3 + i));

        const double measured =
            currentArmQ[i];

        if (!std::isfinite(command) ||
            !std::isfinite(measured))
        {
          RCLCPP_ERROR(
              get_logger(),
              "[SAFETY] Arm joint %zu contains NaN/Inf: "
              "command=%.6f measured=%.6f",
              i + 1,
              command,
              measured);

          armCommand.clear();
          return false;
        }

        const double delta =
            std::abs(command - measured);

        if (delta >
            armMaxDeltaPerStep_)
        {
          RCLCPP_ERROR(
              get_logger(),
              "[SAFETY] Arm joint %zu command jump too large: "
              "command=%.3f measured=%.3f delta=%.3f limit=%.3f",
              i + 1,
              command,
              measured,
              delta,
              armMaxDeltaPerStep_);

          armCommand.clear();
          return false;
        }

        armCommand[i] = command;
      }

      return true;
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(
          get_logger(),
          "[SAFETY] Future policy evaluation failed: %s",
          e.what());

      armCommand.clear();
      return false;
    }
    catch (...)
    {
      RCLCPP_ERROR(
          get_logger(),
          "[SAFETY] Future policy evaluation failed "
          "with an unknown exception.");

      armCommand.clear();
      return false;
    }
  }

  // ---------------------------------------------------------------------------
  // 底盘控制
  // ---------------------------------------------------------------------------
  void publishBaseCommand(
      const ocs2::vector_t &input)
  {
    if (input.size() < 2 ||
        !std::isfinite(input(0)) ||
        !std::isfinite(input(1)))
    {
      publishZeroBaseCommand();
      return;
    }

    if (useStampedCmd_)
    {
      geometry_msgs::msg::TwistStamped msg;

      msg.header.stamp = now();
      msg.header.frame_id = baseFrame_;

      msg.twist.linear.x = input(0);
      msg.twist.angular.z = input(1);

      base_cmd_stamped_pub_->publish(msg);
    }
    else
    {
      geometry_msgs::msg::Twist msg;

      msg.linear.x = input(0);
      msg.angular.z = input(1);

      base_cmd_unstamped_pub_->publish(msg);
    }
  }

  void publishZeroBaseCommand()
  {
    if (useStampedCmd_)
    {
      geometry_msgs::msg::TwistStamped msg;

      msg.header.stamp = now();
      msg.header.frame_id = baseFrame_;

      msg.twist.linear.x = 0.0;
      msg.twist.linear.y = 0.0;
      msg.twist.linear.z = 0.0;

      msg.twist.angular.x = 0.0;
      msg.twist.angular.y = 0.0;
      msg.twist.angular.z = 0.0;

      base_cmd_stamped_pub_->publish(msg);
    }
    else
    {
      geometry_msgs::msg::Twist msg;

      msg.linear.x = 0.0;
      msg.linear.y = 0.0;
      msg.linear.z = 0.0;

      msg.angular.x = 0.0;
      msg.angular.y = 0.0;
      msg.angular.z = 0.0;

      base_cmd_unstamped_pub_->publish(msg);
    }
  }

  // ---------------------------------------------------------------------------
  // ForwardCommandController 机械臂控制
  // ---------------------------------------------------------------------------
  void publishArmPositions(
      const std::vector<double> &positions)
  {
    if (positions.size() != armJointNames_.size())
    {
      RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "Arm command size mismatch: command=%zu, joints=%zu",
          positions.size(),
          armJointNames_.size());

      return;
    }

    for (size_t i = 0;
         i < positions.size();
         ++i)
    {
      if (!std::isfinite(positions[i]))
      {
        RCLCPP_ERROR_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Arm command joint %zu contains NaN/Inf",
            i + 1);

        return;
      }
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data = positions;

    // data[0] -> joint_1
    // data[1] -> joint_2
    // ...
    // 实际映射由控制器 YAML 中 joints 顺序决定。
    arm_cmd_pub_->publish(msg);
  }

  void publishHoldArmCommand()
  {
    std::vector<double> holdCommand;

    if (lastGoodArmQ_.size() ==
        armJointNames_.size())
    {
      holdCommand = lastGoodArmQ_;
    }
    else
    {
      // 如果还没有发出过有效 MPC 命令，则保持当前实测位置。
      std::lock_guard<std::mutex> lock(stateMutex_);
      holdCommand = armQ_;
    }

    if (holdCommand.size() !=
        armJointNames_.size())
    {
      RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "Cannot publish arm hold command: invalid command size.");

      return;
    }

    publishArmPositions(holdCommand);
  }

  // ---------------------------------------------------------------------------
  // Members
  // ---------------------------------------------------------------------------
  std::string taskFile_;
  std::string libFolder_;
  std::string urdfFile_;

  std::string baseFrame_;
  std::string worldFrame_;
  std::string eeFrame_;

  double mrtRate_{100.0};
  double trajHorizon_{0.05};

  bool useStampedCmd_{true};
  bool useWholeBodyTarget_{true};

  double armMaxDeltaPerStep_{0.50};

  std::vector<std::string> armJointNames_;

  bool enableViz_{true};
  bool vizSelfCollision_{true};
  double vizRate_{20.0};

  int vizEveryN_{5};
  long vizCounter_{0};

  std::unique_ptr<
      tracer_jaka::TracerJakaVisualization>
      viz_;

  rclcpp::Time lastReport_;

  size_t loopCount_{0};
  size_t policyUpdateCount_{0};

  double loopWorkSumMs_{0.0};
  double loopWorkMaxMs_{0.0};
  double planAgeSumMs_{0.0};
  double planAgeMaxMs_{0.0};

  std::unique_ptr<
      ocs2::mobile_manipulator::
          MobileManipulatorInterface>
      interface_;

  rclcpp::Node::SharedPtr ocs2InternalNode_;

  std::unique_ptr<
      ocs2::MRT_ROS_Interface>
      mrt_;

  size_t stateDim_{0};
  size_t inputDim_{0};
  size_t armDim_{0};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std::unique_ptr<
      tf2_ros::TransformListener>
      tf_listener_;

  rclcpp::Publisher<
      geometry_msgs::msg::TwistStamped>::SharedPtr
      base_cmd_stamped_pub_;

  rclcpp::Publisher<
      geometry_msgs::msg::Twist>::SharedPtr
      base_cmd_unstamped_pub_;

  // 修改点：ForwardCommandController 发布器
  rclcpp::Publisher<
      std_msgs::msg::Float64MultiArray>::SharedPtr
      arm_cmd_pub_;

  rclcpp::Subscription<
      nav_msgs::msg::Odometry>::SharedPtr
      odom_sub_;

  rclcpp::Subscription<
      sensor_msgs::msg::JointState>::SharedPtr
      js_sub_;

  std::mutex stateMutex_;

  std::atomic<bool> gotOdom_{false};
  std::atomic<bool> gotJs_{false};

  double baseX_{0.0};
  double baseY_{0.0};
  double baseYaw_{0.0};

  std::vector<double> armQ_;
  std::vector<double> lastGoodArmQ_;
};

int main(
    int argc,
    char **argv)
{
  rclcpp::init(argc, argv);

  auto bridge =
      std::make_shared<TracerJakaMrtBridge>();

  try
  {
    bridge->initMrt();
  }
  catch (const std::exception &e)
  {
    RCLCPP_FATAL(
        bridge->get_logger(),
        "initMrt failed: %s",
        e.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(bridge);

  std::thread spinner(
      [&executor]()
      {
        try
        {
          executor.spin();
        }
        catch (const std::exception &)
        {
        }
        catch (...)
        {
        }
      });

  try
  {
    bridge->run();
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(
        bridge->get_logger(),
        "MRT loop exception: %s",
        e.what());
  }
  catch (...)
  {
    RCLCPP_ERROR(
        bridge->get_logger(),
        "MRT loop stopped by an unknown exception.");
  }

  executor.cancel();

  if (spinner.joinable())
  {
    spinner.join();
  }

  bridge->shutdownOcs2();

  rclcpp::shutdown();
  return 0;
}
