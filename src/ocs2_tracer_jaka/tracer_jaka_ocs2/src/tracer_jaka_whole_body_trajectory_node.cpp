// =============================================================================
//  tracer_jaka_whole_body_trajectory_node.cpp
//
//  全身轨迹目标发布节点。
//
//  把一条 CSV 全身轨迹转换成 OCS2 的 TargetTrajectories 发布到
//      <robot_name>_mpc_target
//  由 RosReferenceManager 接收, 再由 WholeBodyTrajectoryCost 读取执行。
//
//  与旧的 EE 轨迹节点的本质区别:
//      旧: stateTrajectory[i] = [x, y, z, qx, qy, qz, qw]        (7 维 EE pose)
//      新: stateTrajectory[i] = [x, y, yaw, q1, ..., q6]         (stateDim 维全身状态)
//
//  CSV 格式 (表头必须存在, 列顺序任意):
//      time,x,y,yaw,q1,q2,q3,q4,q5,q6
//  其中:
//      - time 列可选。缺失时按 linear_speed / angular_speed / joint_speed 自动时间参数化。
//      - 关节列名支持 q1..qN 或 joint_1..joint_N。
//      - yaw 单位 rad; 可以是 wrap 过的值, 节点内部会自动 unwrap 成连续序列。
//
//  时间基准:
//      订阅 <robot_name>_mpc_observation, 取 observation.time 作为 t0,
//      发布 timeTrajectory[i] = t0 + start_lead + relTime[i]。
//      (OCS2 要求 target 时间和 SystemObservation.time 用同一个时钟)
//
//  首点衔接:
//      prepend_current = true 时, 把 observation 里的当前全身状态作为第 0 个航点,
//      并按可行速度给出到第一个 CSV 航点的过渡时间, 避免起步瞬间的目标跳变。
// =============================================================================

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

#include <ocs2_msgs/msg/mpc_observation.hpp>


class WholeBodyTrajectoryTargetNode : public rclcpp::Node {
 public:
  WholeBodyTrajectoryTargetNode() : Node("whole_body_trajectory_target_node") {
    // ── 基础参数 ────────────────────────────────────────────
    declare_parameter<std::string>(
        "csv_file",
        "/home/a/ocs2_ws/outputs/whole_body_path.csv");

    declare_parameter<std::string>("robot_name", "mobile_manipulator");
    declare_parameter<std::string>("world_frame", "odom");

    // 必须和 OCS2 模型一致:
    //   wheelBasedMobileManipulator + 6 轴臂 -> state 9, input 8, base 3, arm 6
    declare_parameter<int>("state_dim", 9);
    declare_parameter<int>("input_dim", 8);
    declare_parameter<int>("base_dim",  3);   // [x, y, yaw]
    declare_parameter<int>("arm_dim",   6);

    // ── 时间参数化 (仅当 CSV 没有 time 列时使用) ─────────────
    declare_parameter<double>("linear_speed",  0.15);   // m/s
    declare_parameter<double>("angular_speed", 0.30);   // rad/s
    declare_parameter<double>("joint_speed",   0.50);   // rad/s
    declare_parameter<double>("min_dt",        0.10);   // s
    declare_parameter<double>("time_scale",    1.0);    // 整体放慢/加快

    // 轨迹起点相对 t0 的提前量 [s]
    declare_parameter<double>("start_lead", 1.0);

    // ── 行为参数 ────────────────────────────────────────────
    declare_parameter<bool>("auto_publish", true);
    // 收到第一帧 observation 之后, 再等这么久才发布 (等 MRT 控制循环稳定)
    declare_parameter<double>("auto_publish_delay", 1.0);
    declare_parameter<bool>("prepend_current_state", true);
    // 轨迹末尾追加一个"保持不动"的航点, 让 MPC 在轨迹结束后停在终点构型
    declare_parameter<double>("hold_time_at_end", 3.0);

    declare_parameter<std::string>("path_topic", "whole_body_target_path");

    // ── 读取参数 ────────────────────────────────────────────
    csvFile_    = get_parameter("csv_file").as_string();
    robotName_  = get_parameter("robot_name").as_string();
    worldFrame_ = get_parameter("world_frame").as_string();

    stateDim_ = get_parameter("state_dim").as_int();
    inputDim_ = get_parameter("input_dim").as_int();
    baseDim_  = get_parameter("base_dim").as_int();
    armDim_   = get_parameter("arm_dim").as_int();

    vLin_       = get_parameter("linear_speed").as_double();
    wAng_       = get_parameter("angular_speed").as_double();
    qDot_       = get_parameter("joint_speed").as_double();
    minDt_      = get_parameter("min_dt").as_double();
    timeScale_  = get_parameter("time_scale").as_double();
    startLead_  = get_parameter("start_lead").as_double();

    autoPublish_      = get_parameter("auto_publish").as_bool();
    autoPublishDelay_ = get_parameter("auto_publish_delay").as_double();
    prependCurrent_   = get_parameter("prepend_current_state").as_bool();
    holdTimeAtEnd_    = get_parameter("hold_time_at_end").as_double();

    pathTopic_ = get_parameter("path_topic").as_string();

    // ── 参数保护 ────────────────────────────────────────────
    if (baseDim_ + armDim_ != stateDim_) {
      RCLCPP_FATAL(get_logger(),
                   "base_dim(%d) + arm_dim(%d) != state_dim(%d)",
                   baseDim_, armDim_, stateDim_);
      throw std::runtime_error("dimension mismatch");
    }
    if (vLin_  <= 1e-6) { vLin_ = 0.15; }
    if (wAng_  <= 1e-6) { wAng_ = 0.30; }
    if (qDot_  <= 1e-6) { qDot_ = 0.50; }
    if (minDt_ <= 1e-6) { minDt_ = 0.05; }
    if (timeScale_ <= 1e-6) { timeScale_ = 1.0; }

    // ── 加载 CSV ────────────────────────────────────────────
    if (!loadCsv(csvFile_)) {
      RCLCPP_ERROR(get_logger(), "读取 CSV 失败或轨迹为空, 节点无法发布轨迹。");
    } else {
      unwrapYaw();
      buildRelativeTimes();
      RCLCPP_INFO(get_logger(),
                  "成功加载 %zu 个全身航点, 轨迹总时长 ≈ %.2f s。",
                  states_.size(),
                  relTimes_.empty() ? 0.0 : relTimes_.back());
    }

    // ── 订阅 MPC observation: 拿时间基准 + 当前全身状态 ───────
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
            haveCurrentState_ = true;
          }
          if (!haveObservation_) {
            firstObsWall_ = now();
          }
          haveObservation_ = true;
        });
    RCLCPP_INFO(get_logger(), "订阅 MPC 观测以对齐时间基准: %s", obsTopic.c_str());

    // ── 可视化 (RViz 里看底盘参考路径) ──────────────────────
    const auto vizQos =
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    pathPub_ = create_publisher<nav_msgs::msg::Path>(pathTopic_, vizQos);
  }

  /// 需要 shared_from_this(), 必须在 main 里单独调用
  void init() {
    targetPub_ = std::make_unique<ocs2::TargetTrajectoriesRosPublisher>(
        shared_from_this(), robotName_);

    publishSrv_ = create_service<std_srvs::srv::Trigger>(
        "/whole_body_trajectory/publish",
        std::bind(&WholeBodyTrajectoryTargetNode::onPublishService, this,
                  std::placeholders::_1, std::placeholders::_2));

    publishPathViz();

    if (autoPublish_) {
      autoTimer_ = create_wall_timer(
          std::chrono::milliseconds(100),
          std::bind(&WholeBodyTrajectoryTargetNode::onAutoTimer, this));
    }

    RCLCPP_INFO(get_logger(),
                "节点就绪。robot=%s stateDim=%d inputDim=%d 航点数=%zu",
                robotName_.c_str(), stateDim_, inputDim_, states_.size());
    RCLCPP_INFO(get_logger(),
                "调用 /whole_body_trajectory/publish 发布轨迹, 或使用 auto_publish。");
  }

 private:
  // =======================================================================
  // 1) 读取 CSV
  // =======================================================================
  static std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
      out.push_back(cell);
    }
    return out;
  }

  static std::string strip(std::string s) {
    s.erase(std::remove_if(s.begin(), s.end(),
                           [](unsigned char c) { return std::isspace(c); }),
            s.end());
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return s;
  }

  bool loadCsv(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
      RCLCPP_ERROR(get_logger(), "无法打开 CSV: %s", path.c_str());
      return false;
    }

    std::string line;
    if (!std::getline(f, line)) {
      RCLCPP_ERROR(get_logger(), "CSV 为空: %s", path.c_str());
      return false;
    }

    const auto header = splitCsv(line);
    auto col = [&](const std::vector<std::string>& names) -> int {
      for (size_t i = 0; i < header.size(); ++i) {
        const std::string h = strip(header[i]);
        for (const auto& n : names) {
          if (h == n) {
            return static_cast<int>(i);
          }
        }
      }
      return -1;
    };

    const int it   = col({"time", "t"});
    const int ix   = col({"x"});
    const int iy   = col({"y"});
    const int iyaw = col({"yaw", "theta"});

    if (ix < 0 || iy < 0 || iyaw < 0) {
      RCLCPP_ERROR(get_logger(), "CSV 表头必须包含 x, y, yaw。");
      return false;
    }

    std::vector<int> iq(armDim_, -1);
    for (int j = 0; j < armDim_; ++j) {
      iq[j] = col({"q" + std::to_string(j + 1),
                   "joint_" + std::to_string(j + 1),
                   "joint" + std::to_string(j + 1)});
      if (iq[j] < 0) {
        RCLCPP_ERROR(get_logger(), "CSV 表头缺少关节列 q%d (或 joint_%d)。",
                     j + 1, j + 1);
        return false;
      }
    }

    hasTimeColumn_ = (it >= 0);

    states_.clear();
    csvTimes_.clear();

    while (std::getline(f, line)) {
      if (line.empty()) {
        continue;
      }
      const auto cells = splitCsv(line);

      int maxIdx = std::max({ix, iy, iyaw, it});
      for (int j = 0; j < armDim_; ++j) {
        maxIdx = std::max(maxIdx, iq[j]);
      }
      if (static_cast<int>(cells.size()) <= maxIdx) {
        RCLCPP_WARN(get_logger(), "跳过列数不足的一行。");
        continue;
      }

      try {
        ocs2::vector_t x(stateDim_);
        x.setZero();
        x(0) = std::stod(cells[ix]);
        x(1) = std::stod(cells[iy]);
        x(2) = std::stod(cells[iyaw]);
        for (int j = 0; j < armDim_; ++j) {
          x(baseDim_ + j) = std::stod(cells[iq[j]]);
        }
        if (!x.allFinite()) {
          RCLCPP_WARN(get_logger(), "跳过含 NaN/Inf 的一行。");
          continue;
        }

        states_.push_back(std::move(x));
        if (hasTimeColumn_) {
          csvTimes_.push_back(std::stod(cells[it]));
        }
      } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "跳过无法解析的一行: %s", e.what());
      }
    }

    return !states_.empty();
  }

  // =======================================================================
  // 2) yaw unwrap: 保证线性插值不会从 +pi 跳到 -pi
  // =======================================================================
  static double wrapToPi(double a) {
    return std::atan2(std::sin(a), std::cos(a));
  }

  void unwrapYaw() {
    for (size_t i = 1; i < states_.size(); ++i) {
      const double prev = states_[i - 1](2);
      const double cur  = states_[i](2);
      states_[i](2) = prev + wrapToPi(cur - prev);
    }
  }

  // =======================================================================
  // 3) 相对时间参数化
  // =======================================================================
  double transitionTime(const ocs2::vector_t& a, const ocs2::vector_t& b) const {
    const double d    = std::hypot(b(0) - a(0), b(1) - a(1));
    const double dyaw = std::abs(wrapToPi(b(2) - a(2)));
    double dq = 0.0;
    for (int j = 0; j < armDim_; ++j) {
      dq = std::max(dq, std::abs(b(baseDim_ + j) - a(baseDim_ + j)));
    }
    return std::max({d / vLin_, dyaw / wAng_, dq / qDot_, minDt_});
  }

  void buildRelativeTimes() {
    relTimes_.assign(states_.size(), 0.0);

    if (hasTimeColumn_) {
      const double t0 = csvTimes_.front();
      for (size_t i = 0; i < states_.size(); ++i) {
        relTimes_[i] = (csvTimes_[i] - t0) * timeScale_;
      }
      // 强制严格递增 (OCS2 插值要求)
      for (size_t i = 1; i < relTimes_.size(); ++i) {
        if (relTimes_[i] <= relTimes_[i - 1]) {
          relTimes_[i] = relTimes_[i - 1] + minDt_;
        }
      }
    } else {
      for (size_t i = 1; i < states_.size(); ++i) {
        relTimes_[i] =
            relTimes_[i - 1] + transitionTime(states_[i - 1], states_[i]) * timeScale_;
      }
    }
  }

  // =======================================================================
  // 4) 构造并发布 TargetTrajectories
  // =======================================================================
  bool buildAndPublishTrajectory() {
    if (states_.empty()) {
      RCLCPP_ERROR(get_logger(), "没有可用航点。");
      return false;
    }
    if (!targetPub_) {
      RCLCPP_ERROR(get_logger(), "targetPub_ 尚未初始化。");
      return false;
    }

    double t0 = 0.0;
    ocs2::vector_t cur;
    bool haveCur = false;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (!haveObservation_) {
        RCLCPP_WARN(get_logger(),
                    "尚未收到 MPC observation, 时间基准回退为 0.0。请确认 %s_mpc_observation。",
                    robotName_.c_str());
      } else {
        t0 = latestObsTime_;
      }
      if (haveCurrentState_) {
        cur = currentState_;
        haveCur = true;
      }
    }

    std::vector<ocs2::vector_t> stateSeq;
    std::vector<double>         timeSeq;
    stateSeq.reserve(states_.size() + 2);
    timeSeq.reserve(states_.size() + 2);

    // ---- 把 CSV 的 yaw 整体平移到当前 yaw 附近, 避免 ±2π 造成的绕圈 ----
    std::vector<ocs2::vector_t> traj = states_;
    if (haveCur) {
      const double yawFirst  = traj.front()(2);
      const double yawTarget = cur(2) + wrapToPi(yawFirst - cur(2));
      const double shift     = yawTarget - yawFirst;
      for (auto& s : traj) {
        s(2) += shift;
      }
    }

    double cumT = startLead_;

    // ---- 首点: 当前全身状态 ----
    if (prependCurrent_ && haveCur) {
      stateSeq.push_back(cur);
      timeSeq.push_back(cumT);
      cumT += transitionTime(cur, traj.front());
    }

    // ---- CSV 航点 ----
    const double base = cumT;
    for (size_t i = 0; i < traj.size(); ++i) {
      stateSeq.push_back(traj[i]);
      timeSeq.push_back(base + relTimes_[i]);
    }

    // ---- 末尾保持 ----
    if (holdTimeAtEnd_ > 1e-6) {
      stateSeq.push_back(traj.back());
      timeSeq.push_back(timeSeq.back() + holdTimeAtEnd_);
    }

    // ---- 严格递增 & 维度检查 ----
    for (size_t i = 1; i < timeSeq.size(); ++i) {
      if (timeSeq[i] <= timeSeq[i - 1]) {
        timeSeq[i] = timeSeq[i - 1] + minDt_;
      }
    }
    for (const auto& s : stateSeq) {
      if (static_cast<int>(s.size()) != stateDim_) {
        RCLCPP_ERROR(get_logger(), "内部错误: 航点维度 %ld != stateDim %d。",
                     static_cast<long>(s.size()), stateDim_);
        return false;
      }
    }

    // ---- 组装 TargetTrajectories ----
    ocs2::scalar_array_t timeTraj;
    ocs2::vector_array_t stateTraj;
    ocs2::vector_array_t inputTraj;

    const size_t M = stateSeq.size();
    timeTraj.reserve(M);
    stateTraj.reserve(M);
    inputTraj.reserve(M);

    for (size_t i = 0; i < M; ++i) {
      timeTraj.push_back(t0 + timeSeq[i]);
      stateTraj.push_back(stateSeq[i]);
      // 现有 QuadraticInputCost 只做输入正则 (不跟踪 desired input),
      // 因此这里发零输入参考即可。
      inputTraj.push_back(ocs2::vector_t::Zero(inputDim_));
    }

    ocs2::TargetTrajectories tt(std::move(timeTraj), std::move(stateTraj),
                                std::move(inputTraj));
    targetPub_->publishTargetTrajectories(tt);

    RCLCPP_INFO(get_logger(),
                "已发布全身轨迹: %zu 个航点, t0=%.3f, 时间跨度 [%.2f, %.2f] s。",
                M, t0, timeSeq.front(), timeSeq.back());

    publishPathViz();
    return true;
  }

  // =======================================================================
  // 5) 可视化: 底盘参考路径
  // =======================================================================
  void publishPathViz() {
    if (states_.empty()) {
      return;
    }
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = worldFrame_;
    path.poses.reserve(states_.size());

    for (const auto& s : states_) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = s(0);
      ps.pose.position.y = s(1);
      ps.pose.position.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, s(2));
      ps.pose.orientation.x = q.x();
      ps.pose.orientation.y = q.y();
      ps.pose.orientation.z = q.z();
      ps.pose.orientation.w = q.w();

      path.poses.push_back(ps);
    }
    pathPub_->publish(path);
  }

  // =======================================================================
  // 回调
  // =======================================================================
  void onPublishService(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    const bool ok = buildAndPublishTrajectory();
    res->success = ok;
    res->message = ok ? "Whole-body trajectory published" : "Publish failed";
  }

  void onAutoTimer() {
    bool ready;
    rclcpp::Time firstObs;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      ready = haveObservation_ && haveCurrentState_;
      firstObs = firstObsWall_;
    }
    if (!ready) {
      return;
    }
    // 等 MRT 控制循环稳定之后再发, 否则 t0 可能还停在初始的 0.0
    if ((now() - firstObs).seconds() < autoPublishDelay_) {
      return;
    }

    if (buildAndPublishTrajectory()) {
      autoTimer_->cancel();
    }
  }

  // =======================================================================
  // 成员
  // =======================================================================
  std::string csvFile_, robotName_, worldFrame_, pathTopic_;

  int stateDim_{9}, inputDim_{8}, baseDim_{3}, armDim_{6};

  double vLin_{0.15}, wAng_{0.30}, qDot_{0.5}, minDt_{0.1}, timeScale_{1.0};
  double startLead_{1.0}, autoPublishDelay_{1.0}, holdTimeAtEnd_{3.0};
  bool   autoPublish_{true}, prependCurrent_{true}, hasTimeColumn_{false};

  std::vector<ocs2::vector_t> states_;
  std::vector<double>         csvTimes_;
  std::vector<double>         relTimes_;

  std::unique_ptr<ocs2::TargetTrajectoriesRosPublisher> targetPub_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr obsSub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr publishSrv_;
  rclcpp::TimerBase::SharedPtr autoTimer_;

  std::mutex mtx_;
  double latestObsTime_{0.0};
  bool   haveObservation_{false};
  bool   haveCurrentState_{false};
  ocs2::vector_t currentState_;
  rclcpp::Time firstObsWall_;
};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WholeBodyTrajectoryTargetNode>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
