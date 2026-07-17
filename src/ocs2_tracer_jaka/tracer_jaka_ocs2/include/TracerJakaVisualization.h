// =============================================================================
//  TracerJakaVisualization.h
//
//  给 Tracer+Jaka 真实/仿真 MRT 管线用的可视化组件 (ROS 2)。
//
//  和官方 MobileManipulatorDummyVisualization 的区别:
//    * 官方是 DummyObserver, 挂在 MRT_ROS_Dummy_Loop 上, 靠 rollout 生成假观测;
//      本组件不依赖 dummy loop, 由你自己的 MRT 主循环每帧调用 update()。
//    * 官方 publishTargetTrajectories 把 target 当 7 维 EE 位姿解析;
//      本组件按【全身轨迹模式】处理 —— target 是 stateDim(=9) 维 [x,y,yaw,q1..q6],
//      对其做 FK 得到参考 EE 路径, 不会把关节角误当四元数。
//    * 本组件【不】发布 joint_states / world->base TF (那些由仿真或
//      robot_state_publisher 提供), 只发布轨迹 marker 和自碰撞距离, 避免冲突。
//
//  发布话题 (默认):
//    /tracer_jaka/optimizedStateTrajectory   visualization_msgs/MarkerArray
//    /tracer_jaka/optimizedPoseTrajectory    geometry_msgs/PoseArray
//    /tracer_jaka/referenceTrajectory        visualization_msgs/MarkerArray
//    /tracer_jaka/environmentObstacles       visualization_msgs/MarkerArray  (静态, 发一次)
//    (+ 自碰撞距离 marker, 由 GeometryInterfaceVisualization 内部发布)
// =============================================================================

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/CommandData.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_oc/oc_data/PrimalSolution.h>

#include <ocs2_mobile_manipulator/ManipulatorModelInfo.h>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

// 自碰撞距离可视化 (与官方 dummy 可视化同一套依赖)。
#include <ocs2_self_collision_visualization/GeometryInterfaceVisualization.h>

namespace tracer_jaka
{
    class TracerJakaVisualization
    {
    public:
        /// @param node        主 ROS 节点 (marker 发布在此节点上)
        /// @param interface   已构造好的 MobileManipulatorInterface
        /// @param worldFrame  轨迹/障碍 marker 所在的 fixed frame (通常 "odom")
        /// @param enableSelfCollision  是否发布自碰撞距离 marker
        TracerJakaVisualization(
            const rclcpp::Node::SharedPtr& node,
            const ocs2::mobile_manipulator::MobileManipulatorInterface& interface,
            std::string worldFrame = "odom",
            bool enableSelfCollision = true);

        /// 每个 MRT 控制周期调用一次 (或按需降频调用)。
        void update(const ocs2::SystemObservation& observation,
                    const ocs2::PrimalSolution& policy,
                    const ocs2::CommandData& command);

        /// 障碍物是静态的, 构造时会自动发一次; 需要时也可外部再调一次。
        void publishObstacles();

    private:
        void setupPublishers();
        void setupSelfCollision(bool enable);

        void publishOptimizedTrajectory(const rclcpp::Time& stamp,
                                        const ocs2::PrimalSolution& policy);
        void publishReferenceTrajectory(const rclcpp::Time& stamp,
                                        const ocs2::TargetTrajectories& target);

        rclcpp::Node::SharedPtr node_;
        ocs2::PinocchioInterface pinocchioInterface_;
        ocs2::mobile_manipulator::ManipulatorModelInfo modelInfo_;
        std::string worldFrame_;
        std::vector<std::string> removeJointNames_;

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr optimizedTrajPub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr        optimizedPosePub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr referenceTrajPub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacleMarkerPub_;

        std::unique_ptr<ocs2::GeometryInterfaceVisualization> geometryVisualization_;
    };
}  // namespace tracer_jaka
