// =============================================================================
//  tracer_jaka_joy_target_node.cpp
//
//  通过游戏手柄 (sensor_msgs/Joy) 在 odom 系下增量控制 OCS2 末端目标位姿,
//  并在 RViz 中显示实际发送的目标位置和姿态:
//
//    1) PoseStamped: 可用 RViz 的 Pose 显示
//    2) MarkerArray: 用三根箭头显示目标坐标系姿态
//
//  与 TracerJakaTargetNode 行为对齐:
//    - robot_name = "mobile_manipulator"
//    - target     = [x, y, z, qx, qy, qz, qw]   (7 维)
//    - input_dim  = 8  (2 底盘 + 6 臂)
//
// =============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

class TracerJakaJoyTargetNode : public rclcpp::Node {
 public:
  TracerJakaJoyTargetNode() : Node("tracer_jaka_joy_target_node") {
    // ---- 基础参数 ----
    declare_parameter<std::string>("robot_name",   "mobile_manipulator");
    declare_parameter<std::string>("marker_frame", "odom");
    declare_parameter<std::string>("ee_frame",     "tool0");
    declare_parameter<int>("input_dim",            8);
    declare_parameter<std::string>("joy_topic",    "/joy");

    // ---- RViz 可视化参数 ----
    declare_parameter<std::string>("target_pose_topic", "joy_target_pose");
    declare_parameter<std::string>("target_axes_topic", "joy_target_axes");

    declare_parameter<double>("target_axis_length", 0.18);          // 三轴箭头长度 [m]
    declare_parameter<double>("target_axis_shaft_diameter", 0.015); // 箭身直径 [m]
    declare_parameter<double>("target_axis_head_diameter", 0.035);  // 箭头直径 [m]

    // ---- 速度 / 死区参数 ----
    declare_parameter<double>("publish_rate",  50.0);   // [Hz] 目标位姿发布频率
    declare_parameter<double>("linear_speed",  0.15);   // [m/s] 摇杆打满时的线速度
    declare_parameter<double>("angular_speed", 0.6);    // [rad/s] 摇杆打满时的角速度
    declare_parameter<double>("deadzone",      0.10);   // 摇杆死区

    // ---- 手柄轴映射 ----
    //   axes[0]=LX  [1]=LY  [2]=LT  [3]=RX  [4]=RY  [5]=RT  [6]=DPadX  [7]=DPadY
    declare_parameter<int>("axis_x",         1);   // 左摇杆 Y -> 末端 x
    declare_parameter<int>("axis_y",         0);   // 左摇杆 X -> 末端 y
    declare_parameter<int>("axis_z",         4);   // 右摇杆 Y -> 末端 z
    declare_parameter<int>("axis_yaw",       3);   // 右摇杆 X -> yaw
    declare_parameter<int>("axis_pitch_pos", 5);   // RT -> pitch +
    declare_parameter<int>("axis_pitch_neg", 2);   // LT -> pitch -
    declare_parameter<int>("axis_roll",      6);   // 十字键 X -> roll

    // ---- 按键映射 ----
    //   buttons[0]=A [1]=B [2]=X [3]=Y [4]=LB [5]=RB [6]=Back [7]=Start
    declare_parameter<int>("button_deadman", 4);   // LB: 必须按住才更新/发布
    declare_parameter<int>("button_reset",   0);   // A : 重置到当前末端位姿
    declare_parameter<int>("button_home",    1);   // B : 回到启动时 home 位姿

    // ---- 工作空间软限位 ----
    declare_parameter<double>("workspace_z_min",       0.05);
    declare_parameter<double>("workspace_z_max",       1.20);
    declare_parameter<double>("workspace_radius_max",  1.50);

    // ---- 读取参数 ----
    robotName_   = get_parameter("robot_name").as_string();
    markerFrame_ = get_parameter("marker_frame").as_string();
    eeFrame_     = get_parameter("ee_frame").as_string();
    inputDim_    = get_parameter("input_dim").as_int();

    targetPoseTopic_ = get_parameter("target_pose_topic").as_string();
    targetAxesTopic_ = get_parameter("target_axes_topic").as_string();

    axisLength_        = get_parameter("target_axis_length").as_double();
    axisShaftDiameter_ = get_parameter("target_axis_shaft_diameter").as_double();
    axisHeadDiameter_  = get_parameter("target_axis_head_diameter").as_double();

    publishRate_ = get_parameter("publish_rate").as_double();
    vMax_        = get_parameter("linear_speed").as_double();
    wMax_        = get_parameter("angular_speed").as_double();
    deadzone_    = get_parameter("deadzone").as_double();

    axX_         = get_parameter("axis_x").as_int();
    axY_         = get_parameter("axis_y").as_int();
    axZ_         = get_parameter("axis_z").as_int();
    axYaw_       = get_parameter("axis_yaw").as_int();
    axPitchPos_  = get_parameter("axis_pitch_pos").as_int();
    axPitchNeg_  = get_parameter("axis_pitch_neg").as_int();
    axRoll_      = get_parameter("axis_roll").as_int();

    btnDeadman_  = get_parameter("button_deadman").as_int();
    btnReset_    = get_parameter("button_reset").as_int();
    btnHome_     = get_parameter("button_home").as_int();

    zMin_ = get_parameter("workspace_z_min").as_double();
    zMax_ = get_parameter("workspace_z_max").as_double();
    rMax_ = get_parameter("workspace_radius_max").as_double();

    if (publishRate_ <= 1e-6) {
      RCLCPP_WARN(get_logger(), "publish_rate <= 0, force set to 50.0 Hz");
      publishRate_ = 50.0;
    }

    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    const auto joyTopic = get_parameter("joy_topic").as_string();

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
        joyTopic,
        rclcpp::SensorDataQoS(),
        std::bind(&TracerJakaJoyTargetNode::onJoy, this, std::placeholders::_1));

    // RViz 可视化 QoS
    // transient_local: RViz 后启动时也能收到最近一次目标
    const auto viz_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    targetPosePub_ =
        create_publisher<geometry_msgs::msg::PoseStamped>(
            targetPoseTopic_, viz_qos);

    targetAxesPub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(
            targetAxesTopic_, viz_qos);
  }

  void init() {
    targetPub_ = std::make_unique<ocs2::TargetTrajectoriesRosPublisher>(
        shared_from_this(), robotName_);

    currentTarget_ = waitAndLookupEEPose();
    initialTarget_ = currentTarget_;
    haveTarget_    = true;

    // 启动时先发布一次目标和 RViz 可视化
    publishTarget();

    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publishRate_));

    timer_ = create_wall_timer(
        period_ns,
        std::bind(&TracerJakaJoyTargetNode::onTimer, this));

    RCLCPP_INFO(get_logger(),
        "Joy target node up. frame=%s robot=%s rate=%.1fHz vMax=%.2f wMax=%.2f",
        markerFrame_.c_str(),
        robotName_.c_str(),
        publishRate_,
        vMax_,
        wMax_);

    RCLCPP_INFO(get_logger(),
        "RViz PoseStamped topic: %s", targetPoseTopic_.c_str());

    RCLCPP_INFO(get_logger(),
        "RViz target axes topic: %s", targetAxesTopic_.c_str());

    RCLCPP_INFO(get_logger(),
        "*** 必须按住 deadman, 默认 LB, 才会持续发送目标 ***");
  }

 private:
  // ------------------------------------------------------------------------
  // TF 工具
  // ------------------------------------------------------------------------

  geometry_msgs::msg::Pose waitAndLookupEEPose() {
    geometry_msgs::msg::Pose pose;

    rclcpp::Rate r(10);

    for (int i = 0; i < 100 && rclcpp::ok(); ++i) {
      if (tf_buffer_->canTransform(
              markerFrame_,
              eeFrame_,
              tf2::TimePointZero,
              tf2::durationFromSec(0.05))) {
        try {
          auto tf = tf_buffer_->lookupTransform(
              markerFrame_,
              eeFrame_,
              tf2::TimePointZero,
              tf2::durationFromSec(0.5));

          pose.position.x  = tf.transform.translation.x;
          pose.position.y  = tf.transform.translation.y;
          pose.position.z  = tf.transform.translation.z;
          pose.orientation = tf.transform.rotation;

          RCLCPP_INFO(get_logger(),
              "Initial EE pose in %s: pos=(%.3f, %.3f, %.3f), quat=(%.3f, %.3f, %.3f, %.3f)",
              markerFrame_.c_str(),
              pose.position.x,
              pose.position.y,
              pose.position.z,
              pose.orientation.x,
              pose.orientation.y,
              pose.orientation.z,
              pose.orientation.w);

          return pose;
        } catch (const tf2::TransformException& ex) {
          RCLCPP_DEBUG(get_logger(), "TF lookup retry: %s", ex.what());
        }
      }

      r.sleep();
    }

    RCLCPP_WARN(get_logger(),
        "TF %s -> %s 未就绪, 使用默认初始目标位姿 (0.5, 0, 0.5).",
        markerFrame_.c_str(),
        eeFrame_.c_str());

    pose.position.x = 0.5;
    pose.position.y = 0.0;
    pose.position.z = 0.5;

    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;

    return pose;
  }

  std::optional<geometry_msgs::msg::Pose> lookupCurrentEE() {
    try {
      auto tf = tf_buffer_->lookupTransform(
          markerFrame_,
          eeFrame_,
          tf2::TimePointZero,
          tf2::durationFromSec(0.05));

      geometry_msgs::msg::Pose p;

      p.position.x  = tf.transform.translation.x;
      p.position.y  = tf.transform.translation.y;
      p.position.z  = tf.transform.translation.z;
      p.orientation = tf.transform.rotation;

      return p;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(get_logger(), "lookupCurrentEE failed: %s", ex.what());
      return std::nullopt;
    }
  }

  // ------------------------------------------------------------------------
  // Joy 工具
  // ------------------------------------------------------------------------

  double applyDeadzone(double v) const {
    if (std::abs(v) < deadzone_) {
      return 0.0;
    }

    const double sign = v > 0.0 ? 1.0 : -1.0;
    return sign * (std::abs(v) - deadzone_) / (1.0 - deadzone_);
  }

  double getAxis(const sensor_msgs::msg::Joy& j, int idx) const {
    if (idx < 0 || idx >= static_cast<int>(j.axes.size())) {
      return 0.0;
    }

    return applyDeadzone(j.axes[idx]);
  }

  // trigger: 一般 Xbox LT/RT 是 [1.0 松开, -1.0 按到底]
  // 这里归一化为 [0, 1]
  double getTrigger(const sensor_msgs::msg::Joy& j, int idx) const {
    if (idx < 0 || idx >= static_cast<int>(j.axes.size())) {
      return 0.0;
    }

    const double v = (1.0 - j.axes[idx]) * 0.5;
    return v < deadzone_ ? 0.0 : v;
  }

  bool getButton(const sensor_msgs::msg::Joy& j, int idx) const {
    if (idx < 0 || idx >= static_cast<int>(j.buttons.size())) {
      return false;
    }

    return j.buttons[idx] != 0;
  }

  // ------------------------------------------------------------------------
  // 回调
  // ------------------------------------------------------------------------

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

    if (!joy || !haveTarget_) {
      return;
    }

    // ---- reset / home 上升沿处理 ----
    const bool reset = getButton(*joy, btnReset_);
    const bool home  = getButton(*joy, btnHome_);

    if (reset && !lastReset_) {
      if (auto cur = lookupCurrentEE()) {
        currentTarget_ = *cur;
        publishTarget();

        RCLCPP_INFO(get_logger(), "Target reset to CURRENT EE pose.");
      } else {
        RCLCPP_WARN(get_logger(), "Reset failed: TF lookup failed.");
      }
    }

    if (home && !lastHome_) {
      currentTarget_ = initialTarget_;
      publishTarget();

      RCLCPP_INFO(get_logger(), "Target reset to HOME pose.");
    }

    lastReset_ = reset;
    lastHome_  = home;

    // ---- deadman ----
    const bool deadman = getButton(*joy, btnDeadman_);

    if (!deadman) {
      // 松开后不再发送新目标
      // RViz 中会保持最后一次发送的目标位姿
      return;
    }

    // ---- 摇杆输入 ----
    const double ax_x    = getAxis(*joy, axX_);
    const double ax_y    = getAxis(*joy, axY_);
    const double ax_z    = getAxis(*joy, axZ_);
    const double ax_yaw  = getAxis(*joy, axYaw_);
    const double ax_roll = getAxis(*joy, axRoll_);

    const double ax_pitch =
        getTrigger(*joy, axPitchPos_) - getTrigger(*joy, axPitchNeg_);

    const double dt = 1.0 / publishRate_;

    // ---- 平移积分：odom / marker_frame 系下直接累加 ----
    Eigen::Vector3d p_cur(
        currentTarget_.position.x,
        currentTarget_.position.y,
        currentTarget_.position.z);

    Eigen::Vector3d dpos(
        ax_x * vMax_ * dt,
        ax_y * vMax_ * dt,
        ax_z * vMax_ * dt);

    Eigen::Vector3d p_new = p_cur + dpos;

    // ---- 工作空间软限位 ----
    p_new.z() = std::clamp(p_new.z(), zMin_, zMax_);

    const double r = std::hypot(p_new.x(), p_new.y());

    if (r > rMax_ && r > 1e-9) {
      p_new.x() *= rMax_ / r;
      p_new.y() *= rMax_ / r;
    }

    // ---- 姿态积分：世界系 / marker_frame 系下角速度增量 ----
    Eigen::Quaterniond q_cur(
        currentTarget_.orientation.w,
        currentTarget_.orientation.x,
        currentTarget_.orientation.y,
        currentTarget_.orientation.z);

    q_cur.normalize();

    Eigen::Vector3d w(
        ax_roll  * wMax_,
        ax_pitch * wMax_,
        ax_yaw   * wMax_);

    Eigen::Quaterniond q_new = q_cur;

    const double w_norm = w.norm();

    if (w_norm > 1e-6) {
      const Eigen::AngleAxisd da(w_norm * dt, w / w_norm);

      // 左乘表示在世界系 / marker_frame 系下施加增量旋转
      q_new = Eigen::Quaterniond(da) * q_cur;
      q_new.normalize();
    }

    currentTarget_.position.x = p_new.x();
    currentTarget_.position.y = p_new.y();
    currentTarget_.position.z = p_new.z();

    currentTarget_.orientation.x = q_new.x();
    currentTarget_.orientation.y = q_new.y();
    currentTarget_.orientation.z = q_new.z();
    currentTarget_.orientation.w = q_new.w();

    publishTarget();
  }

  // ------------------------------------------------------------------------
  // RViz 可视化发布
  // ------------------------------------------------------------------------

  geometry_msgs::msg::Quaternion eigenQuatToMsg(
      const Eigen::Quaterniond& q) const {
    geometry_msgs::msg::Quaternion msg;

    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    msg.w = q.w();

    return msg;
  }

  visualization_msgs::msg::Marker makeAxisMarker(
      int id,
      const std::string& ns,
      const Eigen::Vector3d& p,
      const Eigen::Quaterniond& q,
      float r,
      float g,
      float b) {
    visualization_msgs::msg::Marker marker;

    marker.header.stamp = now();
    marker.header.frame_id = markerFrame_;

    marker.ns = ns;
    marker.id = id;

    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = p.x();
    marker.pose.position.y = p.y();
    marker.pose.position.z = p.z();
    marker.pose.orientation = eigenQuatToMsg(q);

    // ARROW 类型使用 pose 时:
    // scale.x = 箭头总长度
    // scale.y = 箭身直径
    // scale.z = 箭头直径
    marker.scale.x = axisLength_;
    marker.scale.y = axisShaftDiameter_;
    marker.scale.z = axisHeadDiameter_;

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0f;

    marker.lifetime.sec = 0;
    marker.lifetime.nanosec = 0;

    marker.frame_locked = false;

    return marker;
  }

  void publishTargetPose(
      const Eigen::Vector3d& p,
      const Eigen::Quaterniond& q) {
    geometry_msgs::msg::PoseStamped pose_msg;

    pose_msg.header.stamp = now();
    pose_msg.header.frame_id = markerFrame_;

    pose_msg.pose.position.x = p.x();
    pose_msg.pose.position.y = p.y();
    pose_msg.pose.position.z = p.z();

    pose_msg.pose.orientation = eigenQuatToMsg(q);

    targetPosePub_->publish(pose_msg);
  }

  void publishTargetAxes(
      const Eigen::Vector3d& p,
      const Eigen::Quaterniond& q_target) {
    visualization_msgs::msg::MarkerArray marker_array;

    // RViz 的 ARROW marker 默认沿自身局部 +X 方向
    // 所以:
    //   X 轴箭头: q_target
    //   Y 轴箭头: q_target * RotZ(+90deg)
    //   Z 轴箭头: q_target * RotY(-90deg)

    const Eigen::Quaterniond q_x = q_target;

    const Eigen::Quaterniond q_y =
        q_target *
        Eigen::Quaterniond(
            Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));

    const Eigen::Quaterniond q_z =
        q_target *
        Eigen::Quaterniond(
            Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitY()));

    marker_array.markers.push_back(
        makeAxisMarker(0, "joy_target_axes", p, q_x, 1.0f, 0.0f, 0.0f));  // X 红

    marker_array.markers.push_back(
        makeAxisMarker(1, "joy_target_axes", p, q_y, 0.0f, 1.0f, 0.0f));  // Y 绿

    marker_array.markers.push_back(
        makeAxisMarker(2, "joy_target_axes", p, q_z, 0.0f, 0.2f, 1.0f));  // Z 蓝

    targetAxesPub_->publish(marker_array);
  }

  // ------------------------------------------------------------------------
  // OCS2 目标发布
  // ------------------------------------------------------------------------

  void publishTarget() {
    Eigen::Vector3d p(
        currentTarget_.position.x,
        currentTarget_.position.y,
        currentTarget_.position.z);

    Eigen::Quaterniond q(
        currentTarget_.orientation.w,
        currentTarget_.orientation.x,
        currentTarget_.orientation.y,
        currentTarget_.orientation.z);

    q.normalize();

    // ---------- RViz 显示实际发送的目标位置和姿态 ----------
    publishTargetPose(p, q);
    publishTargetAxes(p, q);

    // ---------- OCS2 target ----------
    // mobile_manipulator target = [x, y, z, qx, qy, qz, qw]
    ocs2::vector_t target(7);
    target << p, q.coeffs();

    ocs2::TargetTrajectories tt(
        ocs2::scalar_array_t{0.0},
        ocs2::vector_array_t{target},
        ocs2::vector_array_t{ocs2::vector_t::Zero(inputDim_)});

    targetPub_->publishTargetTrajectories(tt);
  }

  // ------------------------------------------------------------------------
  // 成员变量
  // ------------------------------------------------------------------------

  std::string robotName_;
  std::string markerFrame_;
  std::string eeFrame_;

  int inputDim_{8};

  std::string targetPoseTopic_;
  std::string targetAxesTopic_;

  double axisLength_{0.18};
  double axisShaftDiameter_{0.015};
  double axisHeadDiameter_{0.035};

  double publishRate_{50.0};
  double vMax_{0.15};
  double wMax_{0.6};
  double deadzone_{0.1};

  int axX_{1};
  int axY_{0};
  int axZ_{4};
  int axYaw_{3};
  int axPitchPos_{5};
  int axPitchNeg_{2};
  int axRoll_{6};

  int btnDeadman_{4};
  int btnReset_{0};
  int btnHome_{1};

  double zMin_{0.05};
  double zMax_{1.20};
  double rMax_{1.50};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::unique_ptr<ocs2::TargetTrajectoriesRosPublisher> targetPub_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr targetPosePub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr targetAxesPub_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mtx_;
  sensor_msgs::msg::Joy::ConstSharedPtr lastJoy_;

  geometry_msgs::msg::Pose currentTarget_;
  geometry_msgs::msg::Pose initialTarget_;

  bool haveTarget_{false};
  bool lastReset_{false};
  bool lastHome_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TracerJakaJoyTargetNode>();
  node->init();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
