// =============================================================================
//  TracerJakaMpcNode.cpp
//
//  OCS2 MPC 求解器节点 (ROS 2)。
//  和 ocs2_mobile_manipulator_ros::MobileManipulatorMpcNode 等价, 仅适配 rclcpp。
//
//  发布 :
//    /mobile_manipulator_mpc_policy
//    /mobile_manipulator_mpc_observation_publisher (debug)
//  订阅 :
//    /mobile_manipulator_mpc_target              ← TargetTrajectories
//    /mobile_manipulator_mpc_observation         ← MRT side
// =============================================================================

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>

int main(int argc, char* argv[]) {

  // -- ROS node handle --
  const std::string robotName = "mobile_manipulator";

  rclcpp::init(argc, argv);
  auto nodeHandle = std::make_shared<rclcpp::Node>(robotName + "_mpc");

  // -- params --
  nodeHandle->declare_parameter<std::string>("taskFile", "");
  nodeHandle->declare_parameter<std::string>("libFolder", "");
  nodeHandle->declare_parameter<std::string>("urdfFile", "");

  const auto taskFile  = nodeHandle->get_parameter("taskFile").as_string();
  const auto libFolder = nodeHandle->get_parameter("libFolder").as_string();
  const auto urdfFile  = nodeHandle->get_parameter("urdfFile").as_string();

  if (taskFile.empty() || libFolder.empty() || urdfFile.empty()) {
    RCLCPP_FATAL(nodeHandle->get_logger(),
                 "taskFile / libFolder / urdfFile parameters must all be set.");
    return 1;
  }

  RCLCPP_INFO(nodeHandle->get_logger(), "Task file : %s", taskFile.c_str());
  RCLCPP_INFO(nodeHandle->get_logger(), "Lib folder: %s", libFolder.c_str());
  RCLCPP_INFO(nodeHandle->get_logger(), "URDF file : %s", urdfFile.c_str());

  // -- robot interface --
  // 该接口负责加载任务文件, 并提供 MPC 求解器所需的各种设置和对象。
  ocs2::mobile_manipulator::MobileManipulatorInterface interface(taskFile, libFolder, urdfFile);

  // -- ROS reference manager (listens to TargetTrajectories topic) --
  auto rosReferenceManagerPtr = std::make_shared<ocs2::RosReferenceManager>(
      robotName, interface.getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nodeHandle);

  // -- MPC solver --
  // 该求解器负责求解 MPC 问题, 并返回控制指令。
  ocs2::GaussNewtonDDP_MPC mpc(interface.mpcSettings(),
                               interface.ddpSettings(),
                               interface.getRollout(),
                               interface.getOptimalControlProblem(),
                               interface.getInitializer());
  mpc.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);

  // -- MPC ROS interface (publishes policy, listens to observations) --
  ocs2::MPC_ROS_Interface mpcNode(mpc, robotName);
  mpcNode.launchNodes(nodeHandle);

  RCLCPP_INFO(nodeHandle->get_logger(), "MPC node started successfully.");

  rclcpp::shutdown();
  return 0;
}
