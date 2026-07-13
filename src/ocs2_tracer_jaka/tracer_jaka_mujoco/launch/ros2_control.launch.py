#!/usr/bin/env python3
"""
方式 B（标准 ros2_control 栈，可接 MoveIt / diff_drive_controller / nav2）:
  需先安装 mujoco_ros2_control 插件，并把 urdf/tracer_jaka_zu5.ros2_control.xacro
  里的 <ros2_control> 段合并进 urdf/tracer_jaka_zu5.urdf 的 </robot> 之前。

启动:
  ros2 launch tracer_jaka_mujoco ros2_control.launch.py
控制:
  底盘: ros2 topic pub /base_controller/cmd_vel_unstamped geometry_msgs/Twist "{linear:{x:0.3}, angular:{z:0.5}}"
  机械臂: 通过 /arm_controller 发 JointTrajectory，或接 MoveIt
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():
    pkg = "tracer_jaka_mujoco"
    share = get_package_share_directory(pkg)
    urdf = os.path.join(share, "urdf", "tracer_jaka_zu5.urdf")
    model = os.path.join(share, "models", "tracer_jaka_zu5.xml")
    ctrl = os.path.join(share, "config", "controllers.yaml")

    with open(urdf, "r") as f:
        robot_description = {"robot_description": f.read()}

    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               output="screen",
               parameters=[robot_description, {"use_sim_time": True}])

    mujoco_node = Node(
        package="mujoco_ros2_control", executable="mujoco_ros2_control",
        output="screen",
        parameters=[robot_description, ctrl,
                    {"mujoco_model_path": model}, {"use_sim_time": True}])


    jsb = Node(package="controller_manager", executable="spawner",
               arguments=["joint_state_broadcaster", "-c", "/controller_manager"])
    arm = Node(package="controller_manager", executable="spawner",
               arguments=["arm_controller", "-c", "/controller_manager"])
    base = Node(package="controller_manager", executable="spawner",
                arguments=["base_controller", "-c", "/controller_manager"])

    return LaunchDescription([
        rsp, mujoco_node, jsb,
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm, base])),
    ])
