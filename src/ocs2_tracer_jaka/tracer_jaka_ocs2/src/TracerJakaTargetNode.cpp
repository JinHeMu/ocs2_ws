// =============================================================================
//  tracer_jaka_target_node.cpp
//
//  自定义的 OCS2 目标位姿发布节点。
//  - frame_id 可配 (默认 odom)，与你 TF 根对齐，避免 RViz 反复请求 marker。
//  - 初始位姿通过 TF 查 odom -> gripper_center_link，让 marker 一开机就贴在夹爪上。
//  - 6-DOF 控制 + 右键菜单 "Send target" 发送 TargetTrajectories。
// =============================================================================

#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

using IM       = visualization_msgs::msg::InteractiveMarker;
using IMC      = visualization_msgs::msg::InteractiveMarkerControl;
using IMF      = visualization_msgs::msg::InteractiveMarkerFeedback;
using Marker   = visualization_msgs::msg::Marker;

class TracerJakaTargetNode : public rclcpp::Node {
 public:
  TracerJakaTargetNode() : Node("tracer_jaka_target_node") {
    declare_parameter<std::string>("robot_name",   "mobile_manipulator");
    declare_parameter<std::string>("marker_frame", "odom");
    declare_parameter<std::string>("ee_frame",     "tool0");
    declare_parameter<double>("marker_scale",      0.3);
    // mobile_manipulator (轮式) 的 input 维度 = 2(底盘) + 6(臂) = 8
    declare_parameter<int>("input_dim",            8);

    robotName_    = get_parameter("robot_name").as_string();
    markerFrame_  = get_parameter("marker_frame").as_string();
    eeFrame_      = get_parameter("ee_frame").as_string();
    markerScale_  = get_parameter("marker_scale").as_double();
    inputDim_     = get_parameter("input_dim").as_int();

    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  }

  /// 由于内部要 shared_from_this()，必须在 main 里单独调用。
  void init() {
    targetPub_ = std::make_unique<ocs2::TargetTrajectoriesRosPublisher>(
        shared_from_this(), robotName_);

    server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
        "target_marker", shared_from_this());

    // ---- 初始位姿 = 当前末端位姿 ----
    geometry_msgs::msg::Pose initPose = waitAndLookupEEPose();

    // ---- 构造 marker ----
    auto marker = makeInteractiveMarker(initPose);

    // 右键菜单
    menuHandler_.insert(
        "Send target",
        std::bind(&TracerJakaTargetNode::onMenuSend, this, std::placeholders::_1));

    server_->insert(marker);
    server_->setCallback(
        marker.name,
        std::bind(&TracerJakaTargetNode::onMarkerFeedback, this, std::placeholders::_1));
    menuHandler_.apply(*server_, marker.name);
    server_->applyChanges();

    RCLCPP_INFO(get_logger(),
                "Interactive target marker is up. frame=%s, robot=%s",
                markerFrame_.c_str(), robotName_.c_str());
  }

 private:
  // 等 TF 出来再查；TransformListener 内部有自己的 spinner，不需要主 node 转起来。
  geometry_msgs::msg::Pose waitAndLookupEEPose() {
    geometry_msgs::msg::Pose pose;
    rclcpp::Rate r(10);
    for (int i = 0; i < 80 && rclcpp::ok(); ++i) {
      if (tf_buffer_->canTransform(
              markerFrame_, eeFrame_, tf2::TimePointZero,
              tf2::durationFromSec(0.05))) {
        try {
          auto tf = tf_buffer_->lookupTransform(
              markerFrame_, eeFrame_, tf2::TimePointZero,
              tf2::durationFromSec(0.5));
          pose.position.x = tf.transform.translation.x;
          pose.position.y = tf.transform.translation.y;
          pose.position.z = tf.transform.translation.z;
          pose.orientation = tf.transform.rotation;
          RCLCPP_INFO(
              get_logger(),
              "Initial EE pose in %s: pos=(%.3f, %.3f, %.3f)",
              markerFrame_.c_str(),
              pose.position.x, pose.position.y, pose.position.z);
          return pose;
        } catch (const tf2::TransformException& ex) {
          // 进入下一轮重试
        }
      }
      r.sleep();
    }
    RCLCPP_WARN(get_logger(),
                "TF %s -> %s 未就绪，使用默认初始 marker 位姿。",
                markerFrame_.c_str(), eeFrame_.c_str());
    pose.position.x = 0.5; pose.position.y = 0.0; pose.position.z = 0.5;
    pose.orientation.w = 1.0;
    return pose;
  }

  IM makeInteractiveMarker(const geometry_msgs::msg::Pose& pose) const {
    IM m;
    m.header.frame_id = markerFrame_;          // *** 关键: 与 TF 根一致 ***
    m.header.stamp    = now();
    m.name            = "ee_target";
    m.description     = "OCS2 EE Target (右键 -> Send target 发送)";
    m.scale           = markerScale_;
    m.pose            = pose;

    // 一个小方块，方便看
    Marker box;
    box.type    = Marker::CUBE;
    box.scale.x = box.scale.y = box.scale.z = markerScale_ * 0.3;
    box.color.r = 0.4f; box.color.g = 0.6f; box.color.b = 1.0f; box.color.a = 0.85f;

    IMC visual;
    visual.always_visible    = true;
    visual.interaction_mode  = IMC::MOVE_3D;
    visual.markers.push_back(box);
    m.controls.push_back(visual);

    // 沿三轴的平移 + 绕三轴的旋转
    auto pushAxis = [&](const std::string& tag, double x, double y, double z) {
      IMC c;
      const double n = std::sqrt(1.0 + x * x + y * y + z * z);
      c.orientation.w = 1.0 / n;
      c.orientation.x = x / n;
      c.orientation.y = y / n;
      c.orientation.z = z / n;

      c.name             = "rotate_" + tag;
      c.interaction_mode = IMC::ROTATE_AXIS;
      m.controls.push_back(c);

      c.name             = "move_" + tag;
      c.interaction_mode = IMC::MOVE_AXIS;
      m.controls.push_back(c);
    };
    pushAxis("x", 1.0, 0.0, 0.0);
    pushAxis("z", 0.0, 1.0, 0.0);   // 注意 ROS marker 约定: 这一对实际控制 Z 轴
    pushAxis("y", 0.0, 0.0, 1.0);   // 这一对实际控制 Y 轴

    return m;
  }

  void onMarkerFeedback(const IMF::ConstSharedPtr fb) {
    std::lock_guard<std::mutex> lk(mtx_);
    lastPose_ = fb->pose;
    haveLastPose_ = true;
  }

  void onMenuSend(const IMF::ConstSharedPtr fb) {
    geometry_msgs::msg::Pose pose;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      pose = haveLastPose_ ? lastPose_ : fb->pose;
    }

    Eigen::Vector3d    p(pose.position.x, pose.position.y, pose.position.z);
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                         pose.orientation.y, pose.orientation.z);
    q.normalize();

    // OCS2 mobile_manipulator target = [x, y, z, qx, qy, qz, qw]
    ocs2::vector_t target(7);
    target << p, q.coeffs();

    ocs2::TargetTrajectories tt(
        ocs2::scalar_array_t{0.0},
        ocs2::vector_array_t{target},
        ocs2::vector_array_t{ocs2::vector_t::Zero(inputDim_)});

    targetPub_->publishTargetTrajectories(tt);

    RCLCPP_INFO(get_logger(),
                "Sent target: pos=(%.3f, %.3f, %.3f) quat=(%.3f, %.3f, %.3f, %.3f)",
                p.x(), p.y(), p.z(), q.x(), q.y(), q.z(), q.w());
  }

  // ---- members ----
  std::string robotName_, markerFrame_, eeFrame_;
  double      markerScale_{0.3};
  int         inputDim_{8};

  std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
  interactive_markers::MenuHandler                              menuHandler_;
  std::unique_ptr<ocs2::TargetTrajectoriesRosPublisher>         targetPub_;

  std::mutex mtx_;
  geometry_msgs::msg::Pose lastPose_;
  bool haveLastPose_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TracerJakaTargetNode>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
