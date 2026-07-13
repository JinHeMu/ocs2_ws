// =============================================================================
// tracer_jaka_rectangle_trajectory_target_node.cpp
//
// 发布一段 OCS2 末端矩形参考轨迹，让 mobile_manipulator 末端跟踪:
//
//   target = [x, y, z, qx, qy, qz, qw]
//   input  = Zero(input_dim)
//
// 默认轨迹:
//   - 矩形平面平行于 odom 的 X-Z 平面
//   - X 方向为矩形宽度方向
//   - Z 方向为矩形高度方向
//   - Y 方向保持启动时末端位置不变
//
// 轨迹策略:
//   - 订阅 /mobile_manipulator_mpc_observation，读取 OCS2 当前时间
//   - 周期性发布从当前 MPC time 往后的 reference horizon
//   - RViz 发布 nav_msgs/Path 预览未来轨迹
//   - RViz 发布 PoseStamped 和 MarkerArray 显示当前发送目标姿态
//
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

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <nav_msgs/msg/path.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>

class TracerJakaRectangleTrajectoryTargetNode : public rclcpp::Node {
 public:
  TracerJakaRectangleTrajectoryTargetNode()
      : Node("tracer_jaka_rectangle_trajectory_target_node") {
    // ------------------------------------------------------------------------
    // OCS2 基础参数
    // ------------------------------------------------------------------------
    declare_parameter<std::string>("robot_name", "mobile_manipulator");
    declare_parameter<int>("input_dim", 8);

    // ------------------------------------------------------------------------
    // 坐标系参数
    // ------------------------------------------------------------------------
    declare_parameter<std::string>("marker_frame", "odom");
    declare_parameter<std::string>("ee_frame", "tool0");

    // ------------------------------------------------------------------------
    // 发布参数
    // ------------------------------------------------------------------------
    declare_parameter<double>("publish_rate", 10.0);     // [Hz]
    declare_parameter<double>("horizon", 3.0);           // [s]
    declare_parameter<double>("knot_dt", 0.10);          // [s]
    declare_parameter<double>("reference_delay", 0.05);  // [s]
    declare_parameter<double>("start_delay", 0.5);       // [s]

    // ------------------------------------------------------------------------
    // 矩形轨迹参数
    // ------------------------------------------------------------------------
    declare_parameter<double>("rect_width", 2.0);       // [m]
    declare_parameter<double>("rect_height", 0.30);      // [m]
    declare_parameter<double>("rect_period", 40);      // [s] 一圈时间
    declare_parameter<double>("cycles", 2.0);            // 走几圈
    declare_parameter<bool>("loop", false);              // true: 一直循环

    // ------------------------------------------------------------------------
    // 墙面方向参数
    //
    // 默认:
    //   wall_u_dir = odom X
    //   wall_v_dir = odom Z
    //
    // 所以轨迹在 odom 的 X-Z 平面内。
    // 如果你的墙面不是平行 odom X-Z，可以改这两个方向。
    // ------------------------------------------------------------------------
    declare_parameter<double>("wall_u_dir_x", 0.0);
    declare_parameter<double>("wall_u_dir_y", 0.0);
    declare_parameter<double>("wall_u_dir_z", 0.0);

    declare_parameter<double>("wall_v_dir_x", 0.0);
    declare_parameter<double>("wall_v_dir_y", 0.0);
    declare_parameter<double>("wall_v_dir_z", 1.0);

    // ------------------------------------------------------------------------
    // 工作空间软限位
    // ------------------------------------------------------------------------
    declare_parameter<double>("workspace_z_min", 0.05);
    declare_parameter<double>("workspace_z_max", 1.20);
    declare_parameter<double>("workspace_radius_max", 1.50);

    // ------------------------------------------------------------------------
    // RViz 可视化参数
    // ------------------------------------------------------------------------
    declare_parameter<std::string>("preview_path_topic",
                                   "rectangle_trajectory_preview");
    declare_parameter<std::string>("target_pose_topic",
                                   "rectangle_target_pose");
    declare_parameter<std::string>("target_axes_topic",
                                   "rectangle_target_axes");

    declare_parameter<double>("target_axis_length", 0.18);
    declare_parameter<double>("target_axis_shaft_diameter", 0.015);
    declare_parameter<double>("target_axis_head_diameter", 0.035);

    // ------------------------------------------------------------------------
    // 读取参数
    // ------------------------------------------------------------------------
    robotName_ = get_parameter("robot_name").as_string();
    inputDim_ = get_parameter("input_dim").as_int();

    markerFrame_ = get_parameter("marker_frame").as_string();
    eeFrame_ = get_parameter("ee_frame").as_string();

    publishRate_ = get_parameter("publish_rate").as_double();
    horizon_ = get_parameter("horizon").as_double();
    knotDt_ = get_parameter("knot_dt").as_double();
    referenceDelay_ = get_parameter("reference_delay").as_double();
    startDelay_ = get_parameter("start_delay").as_double();

    rectWidth_ = get_parameter("rect_width").as_double();
    rectHeight_ = get_parameter("rect_height").as_double();
    rectPeriod_ = get_parameter("rect_period").as_double();
    cycles_ = get_parameter("cycles").as_double();
    loop_ = get_parameter("loop").as_bool();

    wallUDir_ = Eigen::Vector3d(
        get_parameter("wall_u_dir_x").as_double(),
        get_parameter("wall_u_dir_y").as_double(),
        get_parameter("wall_u_dir_z").as_double());

    wallVDir_ = Eigen::Vector3d(
        get_parameter("wall_v_dir_x").as_double(),
        get_parameter("wall_v_dir_y").as_double(),
        get_parameter("wall_v_dir_z").as_double());

    zMin_ = get_parameter("workspace_z_min").as_double();
    zMax_ = get_parameter("workspace_z_max").as_double();
    rMax_ = get_parameter("workspace_radius_max").as_double();

    previewPathTopic_ = get_parameter("preview_path_topic").as_string();
    targetPoseTopic_ = get_parameter("target_pose_topic").as_string();
    targetAxesTopic_ = get_parameter("target_axes_topic").as_string();

    axisLength_ = get_parameter("target_axis_length").as_double();
    axisShaftDiameter_ = get_parameter("target_axis_shaft_diameter").as_double();
    axisHeadDiameter_ = get_parameter("target_axis_head_diameter").as_double();

    sanitizeParameters();
    normalizeWallDirections();

    tfBuffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tfListener_ = std::make_unique<tf2_ros::TransformListener>(*tfBuffer_);

    const auto obsTopic = robotName_ + "_mpc_observation";

    observationSub_ = create_subscription<ocs2_msgs::msg::MpcObservation>(
        obsTopic,
        rclcpp::QoS(1),
        std::bind(&TracerJakaRectangleTrajectoryTargetNode::onObservation,
                  this,
                  std::placeholders::_1));

    const auto vizQos =
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    previewPathPub_ =
        create_publisher<nav_msgs::msg::Path>(previewPathTopic_, vizQos);

    targetPosePub_ =
        create_publisher<geometry_msgs::msg::PoseStamped>(
            targetPoseTopic_, vizQos);

    targetAxesPub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(
            targetAxesTopic_, vizQos);
  }

  void init() {
    targetPub_ = std::make_unique<ocs2::TargetTrajectoriesRosPublisher>(
        shared_from_this(), robotName_);

    homePose_ = waitAndLookupEEPose();

    qHome_ = Eigen::Quaterniond(
        homePose_.orientation.w,
        homePose_.orientation.x,
        homePose_.orientation.y,
        homePose_.orientation.z);

    qHome_.normalize();

    haveHome_ = true;

    const auto periodNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publishRate_));

    timer_ = create_wall_timer(
        periodNs,
        std::bind(&TracerJakaRectangleTrajectoryTargetNode::onTimer, this));

    RCLCPP_INFO(get_logger(),
                "Rectangle trajectory node up.");
    RCLCPP_INFO(get_logger(),
                "robot_name=%s, marker_frame=%s, ee_frame=%s",
                robotName_.c_str(),
                markerFrame_.c_str(),
                eeFrame_.c_str());
    RCLCPP_INFO(get_logger(),
                "OCS2 target topic: /%s_mpc_target",
                robotName_.c_str());
    RCLCPP_INFO(get_logger(),
                "OCS2 observation topic: /%s_mpc_observation",
                robotName_.c_str());
    RCLCPP_INFO(get_logger(),
                "rectangle width=%.3f m, height=%.3f m, period=%.3f s",
                rectWidth_,
                rectHeight_,
                rectPeriod_);
    RCLCPP_INFO(get_logger(),
                "wall_u_dir=(%.3f, %.3f, %.3f), wall_v_dir=(%.3f, %.3f, %.3f)",
                wallUDir_.x(),
                wallUDir_.y(),
                wallUDir_.z(),
                wallVDir_.x(),
                wallVDir_.y(),
                wallVDir_.z());
  }

 private:
  static constexpr double PI = 3.14159265358979323846;

  // --------------------------------------------------------------------------
  // 参数检查
  // --------------------------------------------------------------------------

  void sanitizeParameters() {
    if (publishRate_ <= 1e-6) {
      RCLCPP_WARN(get_logger(), "publish_rate <= 0, force set to 10 Hz");
      publishRate_ = 10.0;
    }

    if (knotDt_ <= 1e-6) {
      RCLCPP_WARN(get_logger(), "knot_dt <= 0, force set to 0.1 s");
      knotDt_ = 0.1;
    }

    if (horizon_ < knotDt_) {
      RCLCPP_WARN(get_logger(), "horizon < knot_dt, force horizon = knot_dt");
      horizon_ = knotDt_;
    }

    if (rectWidth_ <= 1e-6) {
      RCLCPP_WARN(get_logger(), "rect_width <= 0, force set to 0.4 m");
      rectWidth_ = 0.4;
    }

    if (rectHeight_ <= 1e-6) {
      RCLCPP_WARN(get_logger(), "rect_height <= 0, force set to 0.3 m");
      rectHeight_ = 0.3;
    }

    if (rectPeriod_ <= 1e-6) {
      RCLCPP_WARN(get_logger(), "rect_period <= 0, force set to 16 s");
      rectPeriod_ = 16.0;
    }

    if (cycles_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "cycles <= 0, force set to 1.0");
      cycles_ = 1.0;
    }

    if (zMax_ < zMin_) {
      std::swap(zMax_, zMin_);
    }
  }

  void normalizeWallDirections() {
    if (wallUDir_.norm() < 1e-9) {
      RCLCPP_WARN(get_logger(),
                  "wall_u_dir norm too small, using odom X.");
      wallUDir_ = Eigen::Vector3d::UnitX();
    }

    wallUDir_.normalize();

    if (wallVDir_.norm() < 1e-9) {
      RCLCPP_WARN(get_logger(),
                  "wall_v_dir norm too small, using odom Z.");
      wallVDir_ = Eigen::Vector3d::UnitZ();
    }

    // 去掉 wallVDir 在 wallUDir 上的投影，让两个方向正交
    wallVDir_ = wallVDir_ - wallVDir_.dot(wallUDir_) * wallUDir_;

    if (wallVDir_.norm() < 1e-9) {
      RCLCPP_WARN(get_logger(),
                  "wall_v_dir parallel to wall_u_dir, using odom Z fallback.");

      wallVDir_ = Eigen::Vector3d::UnitZ()
                  - Eigen::Vector3d::UnitZ().dot(wallUDir_) * wallUDir_;

      if (wallVDir_.norm() < 1e-9) {
        wallVDir_ = Eigen::Vector3d::UnitY()
                    - Eigen::Vector3d::UnitY().dot(wallUDir_) * wallUDir_;
      }
    }

    wallVDir_.normalize();
  }

  // --------------------------------------------------------------------------
  // TF
  // --------------------------------------------------------------------------

  geometry_msgs::msg::Pose waitAndLookupEEPose() {
    geometry_msgs::msg::Pose pose;

    rclcpp::Rate rate(10);

    for (int i = 0; i < 100 && rclcpp::ok(); ++i) {
      if (tfBuffer_->canTransform(
              markerFrame_,
              eeFrame_,
              tf2::TimePointZero,
              tf2::durationFromSec(0.05))) {
        try {
          const auto tf = tfBuffer_->lookupTransform(
              markerFrame_,
              eeFrame_,
              tf2::TimePointZero,
              tf2::durationFromSec(0.5));

          pose.position.x = tf.transform.translation.x;
          pose.position.y = tf.transform.translation.y;
          pose.position.z = tf.transform.translation.z;
          pose.orientation = tf.transform.rotation;

          RCLCPP_INFO(get_logger(),
                      "Home EE pose in %s: pos=(%.3f, %.3f, %.3f), "
                      "quat=(%.3f, %.3f, %.3f, %.3f)",
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

      rate.sleep();
    }

    RCLCPP_WARN(get_logger(),
                "TF %s -> %s not ready, using default home pose.",
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

  // --------------------------------------------------------------------------
  // OCS2 observation
  // --------------------------------------------------------------------------

  void onObservation(const ocs2_msgs::msg::MpcObservation::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);

    latestMpcTime_ = msg->time;
    haveObservation_ = true;

    if (!haveTrajectoryStartTime_) {
      trajectoryStartTime_ = latestMpcTime_ + startDelay_;
      haveTrajectoryStartTime_ = true;

      RCLCPP_INFO(get_logger(),
                  "Rectangle trajectory will start at OCS2 time %.3f",
                  trajectoryStartTime_);
    }
  }

  bool getLatestMpcTime(double& t) {
    std::lock_guard<std::mutex> lock(mtx_);

    if (!haveObservation_) {
      return false;
    }

    t = latestMpcTime_;
    return true;
  }

  // --------------------------------------------------------------------------
  // 轨迹生成
  // --------------------------------------------------------------------------

  Eigen::Vector3d clampWorkspace(const Eigen::Vector3d& pRaw) const {
    Eigen::Vector3d p = pRaw;

    p.z() = std::clamp(p.z(), zMin_, zMax_);

    const double r = std::hypot(p.x(), p.y());

    if (r > rMax_ && r > 1e-9) {
      p.x() *= rMax_ / r;
      p.y() *= rMax_ / r;
    }

    return p;
  }

  Eigen::Vector3d desiredPositionAtTime(double t) const {
    double tau = t - trajectoryStartTime_;

    if (tau < 0.0) {
      tau = 0.0;
    }

    const double w = rectWidth_;
    const double h = rectHeight_;

    const double perimeter = 2.0 * (w + h);
    const double pathSpeed = perimeter / rectPeriod_;

    double sTotal = pathSpeed * tau;

    if (!loop_) {
      const double maxS = cycles_ * perimeter;
      sTotal = std::min(sTotal, maxS);
    }

    double s = std::fmod(sTotal, perimeter);

    if (s < 0.0) {
      s += perimeter;
    }

    const Eigen::Vector3d p0(
        homePose_.position.x,
        homePose_.position.y,
        homePose_.position.z);

    Eigen::Vector3d p = p0;

    if (s < w) {
      // P0 -> P1
      const double a = s / w;
      p = p0 + a * w * wallUDir_;

    } else if (s < w + h) {
      // P1 -> P2
      const double a = (s - w) / h;
      p = p0 + w * wallUDir_ + a * h * wallVDir_;

    } else if (s < 2.0 * w + h) {
      // P2 -> P3
      const double a = (s - w - h) / w;
      p = p0 + (1.0 - a) * w * wallUDir_ + h * wallVDir_;

    } else {
      // P3 -> P0
      const double a = (s - 2.0 * w - h) / h;
      p = p0 + (1.0 - a) * h * wallVDir_;
    }

    return clampWorkspace(p);
  }

  Eigen::Quaterniond desiredOrientationAtTime(double /*t*/) const {
    // 姿态保持启动时末端姿态不变。
    // 如果需要工具始终朝向墙面，先手动把末端调整到合适姿态，再启动该节点。
    return qHome_;
  }

  ocs2::vector_t makeTargetVector(
      const Eigen::Vector3d& p,
      const Eigen::Quaterniond& qRaw) const {
    Eigen::Quaterniond q = qRaw;
    q.normalize();

    ocs2::vector_t target(7);

    target << p.x(),
              p.y(),
              p.z(),
              q.x(),
              q.y(),
              q.z(),
              q.w();

    return target;
  }

  // --------------------------------------------------------------------------
  // RViz 可视化
  // --------------------------------------------------------------------------

  geometry_msgs::msg::Quaternion eigenQuatToMsg(
      const Eigen::Quaterniond& qRaw) const {
    Eigen::Quaterniond q = qRaw;
    q.normalize();

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

    // ARROW 使用 pose 时:
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
    geometry_msgs::msg::PoseStamped poseMsg;

    poseMsg.header.stamp = now();
    poseMsg.header.frame_id = markerFrame_;

    poseMsg.pose.position.x = p.x();
    poseMsg.pose.position.y = p.y();
    poseMsg.pose.position.z = p.z();

    poseMsg.pose.orientation = eigenQuatToMsg(q);

    targetPosePub_->publish(poseMsg);
  }

  void publishTargetAxes(
      const Eigen::Vector3d& p,
      const Eigen::Quaterniond& qTarget) {
    visualization_msgs::msg::MarkerArray markerArray;

    // RViz ARROW 默认沿自身局部 +X 方向。
    //
    // X 轴箭头: qTarget
    // Y 轴箭头: qTarget * RotZ(+90deg)
    // Z 轴箭头: qTarget * RotY(-90deg)

    const Eigen::Quaterniond qX = qTarget;

    const Eigen::Quaterniond qY =
        qTarget *
        Eigen::Quaterniond(
            Eigen::AngleAxisd(PI / 2.0, Eigen::Vector3d::UnitZ()));

    const Eigen::Quaterniond qZ =
        qTarget *
        Eigen::Quaterniond(
            Eigen::AngleAxisd(-PI / 2.0, Eigen::Vector3d::UnitY()));

    markerArray.markers.push_back(
        makeAxisMarker(0, "rectangle_target_axes", p, qX,
                       1.0f, 0.0f, 0.0f));

    markerArray.markers.push_back(
        makeAxisMarker(1, "rectangle_target_axes", p, qY,
                       0.0f, 1.0f, 0.0f));

    markerArray.markers.push_back(
        makeAxisMarker(2, "rectangle_target_axes", p, qZ,
                       0.0f, 0.2f, 1.0f));

    targetAxesPub_->publish(markerArray);
  }

  // --------------------------------------------------------------------------
  // 主定时器：发布 OCS2 TargetTrajectories
  // --------------------------------------------------------------------------

  void onTimer() {
    if (!haveHome_ || !haveTrajectoryStartTime_) {
      return;
    }

    double currentMpcTime = 0.0;

    if (!getLatestMpcTime(currentMpcTime)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "No OCS2 observation received yet. Waiting for /%s_mpc_observation ...",
          robotName_.c_str());
      return;
    }

    const double t0 = currentMpcTime + referenceDelay_;

    ocs2::scalar_array_t timeTrajectory;
    ocs2::vector_array_t stateTrajectory;
    ocs2::vector_array_t inputTrajectory;

    const int numKnots =
        std::max(2, static_cast<int>(std::ceil(horizon_ / knotDt_)) + 1);

    timeTrajectory.reserve(numKnots);
    stateTrajectory.reserve(numKnots);
    inputTrajectory.reserve(numKnots);

    nav_msgs::msg::Path previewPath;
    previewPath.header.stamp = now();
    previewPath.header.frame_id = markerFrame_;
    previewPath.poses.reserve(numKnots);

    Eigen::Vector3d firstP = Eigen::Vector3d::Zero();
    Eigen::Quaterniond firstQ = qHome_;

    for (int i = 0; i < numKnots; ++i) {
      const double t = t0 + static_cast<double>(i) * knotDt_;

      const Eigen::Vector3d p = desiredPositionAtTime(t);
      const Eigen::Quaterniond q = desiredOrientationAtTime(t);

      if (i == 0) {
        firstP = p;
        firstQ = q;
      }

      timeTrajectory.push_back(t);
      stateTrajectory.push_back(makeTargetVector(p, q));
      inputTrajectory.push_back(ocs2::vector_t::Zero(inputDim_));

      geometry_msgs::msg::PoseStamped pose;
      pose.header = previewPath.header;

      pose.pose.position.x = p.x();
      pose.pose.position.y = p.y();
      pose.pose.position.z = p.z();
      pose.pose.orientation = eigenQuatToMsg(q);

      previewPath.poses.push_back(pose);
    }

    ocs2::TargetTrajectories targetTrajectories(
        timeTrajectory,
        stateTrajectory,
        inputTrajectory);

    targetPub_->publishTargetTrajectories(targetTrajectories);

    previewPathPub_->publish(previewPath);
    publishTargetPose(firstP, firstQ);
    publishTargetAxes(firstP, firstQ);
  }

  // --------------------------------------------------------------------------
  // 成员变量
  // --------------------------------------------------------------------------

  std::string robotName_;
  std::string markerFrame_;
  std::string eeFrame_;

  std::string previewPathTopic_;
  std::string targetPoseTopic_;
  std::string targetAxesTopic_;

  int inputDim_{8};

  double publishRate_{10.0};
  double horizon_{3.0};
  double knotDt_{0.10};
  double referenceDelay_{0.05};
  double startDelay_{0.5};

  double rectWidth_{0.40};
  double rectHeight_{0.30};
  double rectPeriod_{16.0};
  double cycles_{1.0};
  bool loop_{false};

  Eigen::Vector3d wallUDir_{1.0, 0.0, 0.0};
  Eigen::Vector3d wallVDir_{0.0, 0.0, 1.0};

  double zMin_{0.05};
  double zMax_{1.20};
  double rMax_{1.50};

  double axisLength_{0.18};
  double axisShaftDiameter_{0.015};
  double axisHeadDiameter_{0.035};

  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
  std::unique_ptr<tf2_ros::TransformListener> tfListener_;

  std::unique_ptr<ocs2::TargetTrajectoriesRosPublisher> targetPub_;

  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr observationSub_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr previewPathPub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr targetPosePub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr targetAxesPub_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mtx_;
  double latestMpcTime_{0.0};
  bool haveObservation_{false};

  double trajectoryStartTime_{0.0};
  bool haveTrajectoryStartTime_{false};

  geometry_msgs::msg::Pose homePose_;
  Eigen::Quaterniond qHome_{1.0, 0.0, 0.0, 0.0};
  bool haveHome_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TracerJakaRectangleTrajectoryTargetNode>();
  node->init();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
