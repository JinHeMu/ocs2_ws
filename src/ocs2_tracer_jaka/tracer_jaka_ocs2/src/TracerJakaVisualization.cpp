// =============================================================================
//  TracerJakaVisualization.cpp
// =============================================================================

#include <pinocchio/fwd.hpp>  // 必须最先包含

#include "TracerJakaVisualization.h"

#include <algorithm>
#include <array>
#include <sstream>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>

#include <ocs2_mobile_manipulator/AccessHelperFunctions.h>
#include <ocs2_mobile_manipulator/FactoryFunctions.h>

#include <ocs2_ros_interfaces/common/RosMsgHelpers.h>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace
{
    template <typename It>
    void assignHeader(It firstIt, It lastIt, const std_msgs::msg::Header& header)
    {
        for (; firstIt != lastIt; ++firstIt) { firstIt->header = header; }
    }

    template <typename It>
    void assignIncreasingId(It firstIt, It lastIt, int startId = 0)
    {
        for (; firstIt != lastIt; ++firstIt) { firstIt->id = startId++; }
    }

    // 从 property_tree 子节点里读若干个数值 (忽略 key, 只按顺序取 value)
    std::vector<double> readNumbers(const boost::property_tree::ptree& node,
                                    const std::string& childKey)
    {
        std::vector<double> out;
        auto child = node.get_child_optional(childKey);
        if (child) {
            for (const auto& kv : child.get()) {
                try { out.push_back(std::stod(kv.second.data())); }
                catch (...) { /* skip */ }
            }
        }
        return out;
    }
}  // namespace

namespace tracer_jaka
{
    using namespace ocs2;
    using namespace ocs2::mobile_manipulator;

    TracerJakaVisualization::TracerJakaVisualization(
        const rclcpp::Node::SharedPtr& node,
        const MobileManipulatorInterface& interface,
        std::string worldFrame,
        bool enableSelfCollision)
        : node_(node),
          pinocchioInterface_(interface.getPinocchioInterface()),
          modelInfo_(interface.getManipulatorModelInfo()),
          worldFrame_(std::move(worldFrame))
    {
        setupPublishers();
        setupSelfCollision(enableSelfCollision);
        publishObstacles();   // 静态障碍物, 发一次

        RCLCPP_INFO(node_->get_logger(),
                    "[TracerJakaVisualization] up. frame=%s, stateDim=%zu, eeFrame=%s",
                    worldFrame_.c_str(),
                    static_cast<size_t>(modelInfo_.stateDim),
                    modelInfo_.eeFrame.c_str());
    }

    void TracerJakaVisualization::setupPublishers()
    {
        optimizedTrajPub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/tracer_jaka/optimizedStateTrajectory", 1);
        optimizedPosePub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>(
            "/tracer_jaka/optimizedPoseTrajectory", 1);
        referenceTrajPub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/tracer_jaka/referenceTrajectory", 1);
        // 障碍物是静态的, 用 transient_local 让后接入的 RViz 也能收到最后一帧
        obstacleMarkerPub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/tracer_jaka/environmentObstacles",
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    }

    void TracerJakaVisualization::setupSelfCollision(bool enable)
    {
        if (!enable) { return; }

        std::string taskFile, urdfFile;
        try {
            taskFile = node_->get_parameter("taskFile").as_string();
            urdfFile = node_->get_parameter("urdfFile").as_string();
        } catch (const std::exception& e) {
            RCLCPP_WARN(node_->get_logger(),
                        "[TracerJakaVisualization] 读取 taskFile/urdfFile 参数失败: %s; "
                        "跳过自碰撞可视化。", e.what());
            return;
        }

        boost::property_tree::ptree pt;
        boost::property_tree::read_info(taskFile, pt);

        bool activateSelfCollision = true;
        loadData::loadPtreeValue(pt, activateSelfCollision, "selfCollision.activate", true);
        if (!activateSelfCollision) { return; }

        const ManipulatorModelType modelType =
            loadManipulatorType(taskFile, "model_information.manipulatorModelType");
        loadData::loadStdVector<std::string>(
            taskFile, "model_information.removeJoints", removeJointNames_, false);

        std::vector<std::pair<size_t, size_t>>            collisionObjectPairs;
        std::vector<std::pair<std::string, std::string>>  collisionLinkPairs;
        loadData::loadStdVectorOfPair(taskFile, "selfCollision.collisionObjectPairs",
                                      collisionObjectPairs, true);
        loadData::loadStdVectorOfPair(taskFile, "selfCollision.collisionLinkPairs",
                                      collisionLinkPairs, true);

        scalar_t minimumDistance = 0.0;
        scalar_t activationDistance = -1.0;
        loadData::loadPtreeValue(pt, minimumDistance, "selfCollision.minimumDistance", false);
        loadData::loadPtreeValue(pt, activationDistance, "selfCollision.activationDistance", false);
        if (activationDistance < 0.0) { activationDistance = 5.0 * minimumDistance; }

        try {
            PinocchioInterface pinocchioInterface(
                createPinocchioInterface(urdfFile, modelType, removeJointNames_));
            PinocchioGeometryInterface geomInterface(
                pinocchioInterface, urdfFile, collisionLinkPairs, collisionObjectPairs);

            // 与官方 MobileManipulatorDummyVisualization 完全一致的构造调用。
            geometryVisualization_ = std::make_unique<GeometryInterfaceVisualization>(
                std::move(pinocchioInterface), geomInterface, worldFrame_, activationDistance);

            RCLCPP_INFO(node_->get_logger(),
                        "[TracerJakaVisualization] 自碰撞距离可视化已启用 (activationDistance=%.3f)。",
                        activationDistance);
        } catch (const std::exception& e) {
            RCLCPP_WARN(node_->get_logger(),
                        "[TracerJakaVisualization] 初始化自碰撞可视化失败: %s; 已跳过。",
                        e.what());
            geometryVisualization_.reset();
        }
    }

    // =======================================================================
    // 障碍物可视化: 直接解析 task 文件 environmentCollision.obstacles,
    // 和 MobileManipulatorInterface::loadInitialObstacles 用同一份配置。
    // =======================================================================
    void TracerJakaVisualization::publishObstacles()
    {
        if (!obstacleMarkerPub_) { return; }

        std::string taskFile;
        try { taskFile = node_->get_parameter("taskFile").as_string(); }
        catch (...) { return; }

        boost::property_tree::ptree pt;
        try { boost::property_tree::read_info(taskFile, pt); }
        catch (const std::exception& e) {
            RCLCPP_WARN(node_->get_logger(),
                        "[TracerJakaVisualization] 解析 task 文件失败: %s", e.what());
            return;
        }

        bool envActivate = false;
        loadData::loadPtreeValue(pt, envActivate, "environmentCollision.activate", false);
        if (!envActivate) {
            RCLCPP_INFO(node_->get_logger(),
                        "[TracerJakaVisualization] environmentCollision.activate=false, 无障碍物可视化。");
            return;
        }

        auto obstaclesOpt = pt.get_child_optional("environmentCollision.obstacles");
        if (!obstaclesOpt) { return; }

        const rclcpp::Time stamp = node_->now();
        visualization_msgs::msg::MarkerArray arr;

        // 先 DELETEALL
        {
            visualization_msgs::msg::Marker del;
            del.header.frame_id = worldFrame_;
            del.header.stamp = stamp;
            del.action = visualization_msgs::msg::Marker::DELETEALL;
            arr.markers.push_back(del);
        }

        int id = 0;
        int count = 0;
        for (const auto& obsPair : obstaclesOpt.get()) {
            const std::string& name = obsPair.first;
            const auto& node = obsPair.second;

            const std::string type = node.get<std::string>("type", "");
            if (type.empty()) { continue; }

            const auto pos = readNumbers(node, "position");
            const auto ori = readNumbers(node, "orientation");  // w, x, y, z

            visualization_msgs::msg::Marker m;
            m.header.frame_id = worldFrame_;
            m.header.stamp = stamp;
            m.ns = "env_obstacles";
            m.id = id++;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = pos.size() > 0 ? pos[0] : 0.0;
            m.pose.position.y = pos.size() > 1 ? pos[1] : 0.0;
            m.pose.position.z = pos.size() > 2 ? pos[2] : 0.0;
            if (ori.size() == 4) {
                m.pose.orientation.w = ori[0];
                m.pose.orientation.x = ori[1];
                m.pose.orientation.y = ori[2];
                m.pose.orientation.z = ori[3];
            } else {
                m.pose.orientation.w = 1.0;
            }
            m.color.r = 0.85f; m.color.g = 0.20f; m.color.b = 0.20f; m.color.a = 0.55f;

            if (type == "box") {
                const auto he = readNumbers(node, "halfExtents");
                m.type = visualization_msgs::msg::Marker::CUBE;
                m.scale.x = 2.0 * (he.size() > 0 ? he[0] : 0.1);
                m.scale.y = 2.0 * (he.size() > 1 ? he[1] : 0.1);
                m.scale.z = 2.0 * (he.size() > 2 ? he[2] : 0.1);
            } else if (type == "sphere") {
                const double r = node.get<double>("radius", 0.1);
                m.type = visualization_msgs::msg::Marker::SPHERE;
                m.scale.x = m.scale.y = m.scale.z = 2.0 * r;
            } else if (type == "cylinder") {
                const double r = node.get<double>("radius", 0.1);
                const double h = node.get<double>("height", 0.2);
                m.type = visualization_msgs::msg::Marker::CYLINDER;
                m.scale.x = m.scale.y = 2.0 * r;
                m.scale.z = h;
            } else {
                continue;
            }
            arr.markers.push_back(m);

            // 文字标签
            visualization_msgs::msg::Marker txt;
            txt.header.frame_id = worldFrame_;
            txt.header.stamp = stamp;
            txt.ns = "env_obstacle_labels";
            txt.id = id++;
            txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            txt.action = visualization_msgs::msg::Marker::ADD;
            txt.pose.position.x = m.pose.position.x;
            txt.pose.position.y = m.pose.position.y;
            txt.pose.position.z = m.pose.position.z + 0.5 * m.scale.z + 0.1;
            txt.pose.orientation.w = 1.0;
            txt.scale.z = 0.08;
            txt.color.r = txt.color.g = txt.color.b = 1.0f; txt.color.a = 1.0f;
            txt.text = name + " (" + type + ")";
            arr.markers.push_back(txt);

            ++count;
        }

        obstacleMarkerPub_->publish(arr);
        RCLCPP_INFO(node_->get_logger(),
                    "[TracerJakaVisualization] 已发布 %d 个环境障碍物 marker 到 "
                    "/tracer_jaka/environmentObstacles (frame=%s)。",
                    count, worldFrame_.c_str());
    }

    void TracerJakaVisualization::update(const SystemObservation& observation,
                                        const PrimalSolution& policy,
                                        const CommandData& command)
    {
        const rclcpp::Time stamp = node_->get_clock()->now();

        publishOptimizedTrajectory(stamp, policy);
        publishReferenceTrajectory(stamp, command.mpcTargetTrajectories_);

        if (geometryVisualization_ != nullptr) {
            geometryVisualization_->publishDistances(observation.state);
        }
    }

    void TracerJakaVisualization::publishOptimizedTrajectory(const rclcpp::Time& stamp,
                                                            const PrimalSolution& policy)
    {
        const scalar_t TRAJECTORYLINEWIDTH = 0.005;
        const std::array<scalar_t, 3> red{0.6350, 0.0780, 0.1840};
        const std::array<scalar_t, 3> blue{0.0, 0.4470, 0.7410};
        const auto& mpcStateTrajectory = policy.stateTrajectory_;
        if (mpcStateTrajectory.empty()) { return; }

        visualization_msgs::msg::MarkerArray markerArray;

        const auto& model = pinocchioInterface_.getModel();
        auto& data = pinocchioInterface_.getData();

        // EE 预测轨迹 (蓝线, FK)
        std::vector<geometry_msgs::msg::Point> endEffectorTrajectory;
        endEffectorTrajectory.reserve(mpcStateTrajectory.size());
        std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(),
                      [&](const vector_t& state)
                      {
                          pinocchio::forwardKinematics(model, data, state);
                          pinocchio::updateFramePlacements(model, data);
                          const auto eeIndex = model.getBodyId(modelInfo_.eeFrame);
                          const vector_t eePosition = data.oMf[eeIndex].translation();
                          endEffectorTrajectory.push_back(ros_msg_helpers::getPointMsg(eePosition));
                      });
        markerArray.markers.emplace_back(ros_msg_helpers::getLineMsg(
            std::move(endEffectorTrajectory), blue, TRAJECTORYLINEWIDTH));
        markerArray.markers.back().ns = "EE Trajectory";

        // 底盘预测轨迹 (红线) + 位姿数组
        std::vector<geometry_msgs::msg::Point> baseTrajectory;
        baseTrajectory.reserve(mpcStateTrajectory.size());
        geometry_msgs::msg::PoseArray poseArray;
        poseArray.poses.reserve(mpcStateTrajectory.size());
        std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(),
                      [&](const vector_t& state)
                      {
                          const auto r_world_base = getBasePosition(state, modelInfo_);
                          const Eigen::Quaternion<scalar_t> q_world_base =
                              getBaseOrientation(state, modelInfo_);
                          geometry_msgs::msg::Pose pose;
                          pose.position = ros_msg_helpers::getPointMsg(r_world_base);
                          pose.orientation = ros_msg_helpers::getOrientationMsg(q_world_base);
                          baseTrajectory.push_back(pose.position);
                          poseArray.poses.push_back(std::move(pose));
                      });
        markerArray.markers.emplace_back(ros_msg_helpers::getLineMsg(
            std::move(baseTrajectory), red, TRAJECTORYLINEWIDTH));
        markerArray.markers.back().ns = "Base Trajectory";

        assignHeader(markerArray.markers.begin(), markerArray.markers.end(),
                     ros_msg_helpers::getHeaderMsg(worldFrame_, stamp));
        assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());
        poseArray.header = ros_msg_helpers::getHeaderMsg(worldFrame_, stamp);

        optimizedTrajPub_->publish(markerArray);
        optimizedPosePub_->publish(poseArray);
    }

    void TracerJakaVisualization::publishReferenceTrajectory(
        const rclcpp::Time& stamp, const TargetTrajectories& target)
    {
        const auto& refStates = target.stateTrajectory;
        if (refStates.empty()) { return; }

        const scalar_t LINEWIDTH = 0.006;
        const std::array<scalar_t, 3> green{0.10, 0.75, 0.20};
        const std::array<scalar_t, 3> orange{0.95, 0.60, 0.10};

        visualization_msgs::msg::MarkerArray markerArray;
        const int refDim = static_cast<int>(refStates.front().size());

        if (refDim == modelInfo_.stateDim)
        {
            const auto& model = pinocchioInterface_.getModel();
            auto& data = pinocchioInterface_.getData();

            std::vector<geometry_msgs::msg::Point> eeRef, baseRef;
            eeRef.reserve(refStates.size());
            baseRef.reserve(refStates.size());
            for (const auto& state : refStates)
            {
                pinocchio::forwardKinematics(model, data, state);
                pinocchio::updateFramePlacements(model, data);
                const auto eeIndex = model.getBodyId(modelInfo_.eeFrame);
                eeRef.push_back(ros_msg_helpers::getPointMsg(
                    vector_t(data.oMf[eeIndex].translation())));
                baseRef.push_back(ros_msg_helpers::getPointMsg(
                    getBasePosition(state, modelInfo_)));
            }
            markerArray.markers.emplace_back(
                ros_msg_helpers::getLineMsg(std::move(eeRef), green, LINEWIDTH));
            markerArray.markers.back().ns = "Reference EE Trajectory";
            markerArray.markers.emplace_back(
                ros_msg_helpers::getLineMsg(std::move(baseRef), orange, LINEWIDTH));
            markerArray.markers.back().ns = "Reference Base Trajectory";
        }
        else if (refDim >= 7)
        {
            std::vector<geometry_msgs::msg::Point> eeRef;
            eeRef.reserve(refStates.size());
            for (const auto& state : refStates) {
                eeRef.push_back(ros_msg_helpers::getPointMsg(vector_t(state.head(3))));
            }
            markerArray.markers.emplace_back(
                ros_msg_helpers::getLineMsg(std::move(eeRef), green, LINEWIDTH));
            markerArray.markers.back().ns = "Reference EE Trajectory";
        }
        else
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                 "[TracerJakaVisualization] 参考 target 维度 %d 无法识别, 跳过。",
                                 refDim);
            return;
        }

        assignHeader(markerArray.markers.begin(), markerArray.markers.end(),
                     ros_msg_helpers::getHeaderMsg(worldFrame_, stamp));
        assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());
        referenceTrajPub_->publish(markerArray);
    }
}  // namespace tracer_jaka
