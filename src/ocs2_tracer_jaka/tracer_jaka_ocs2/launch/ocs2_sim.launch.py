#!/usr/bin/env python3
# =============================================================================
#  ocs2_mujoco_sim.launch.py
#
#  MuJoCo + ros2_control + OCS2 集成 launch:
#    1. 准备 OCS2 需要的 URDF 文件路径
#       - 如果输入是 .xacro，则编译成 /tmp/.../tracer_jaka.urdf
#       - 如果输入是 .urdf，则直接复制到 /tmp/.../tracer_jaka.urdf
#    2. 启动 tracer_jaka_mujoco/ros2_control.launch.py
#       - robot_state_publisher
#       - mujoco_ros2_control
#       - joint_state_broadcaster
#       - arm_controller
#       - base_controller
#    3. 等 MuJoCo 和控制器就绪后，启动 OCS2:
#       - tracer_jaka_mpc_node
#       - tracer_jaka_mrt_node
#       - tracer_jaka_target_node
#       - 可选 joy 控制目标点
#    4. 可选 RViz2
# =============================================================================

import os
import shutil

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _ensure_urdf(context, *args, **kwargs):
    """
    给 OCS2 准备一个实际存在的 URDF 文件路径。

    OCS2 通常需要 urdfFile 是一个真实 .urdf 路径，
    不能直接吃 xacro。
    """
    src = context.perform_substitution(LaunchConfiguration("xacro_file"))
    dst = context.perform_substitution(LaunchConfiguration("urdf_file"))

    os.makedirs(os.path.dirname(dst), exist_ok=True)

    # 如果是 xacro，就编译
    if src.endswith(".xacro"):
        return [
            LogInfo(msg=f"[ocs2_mujoco] xacro -> urdf: {src} -> {dst}"),
            ExecuteProcess(
                cmd=[
                    "xacro",
                    src,
                    "sim_mode:=true",
                    "-o",
                    dst,
                ],
                output="screen",
                shell=False,
            ),
        ]

    # 如果已经是 urdf，就直接复制到 /tmp 给 OCS2 用
    if os.path.abspath(src) != os.path.abspath(dst):
        shutil.copyfile(src, dst)

    return [
        LogInfo(msg=f"[ocs2_mujoco] use urdf: {dst}")
    ]


def generate_launch_description():
    pkg_ocs2 = FindPackageShare("tracer_jaka_ocs2")
    pkg_mujoco = FindPackageShare("tracer_jaka_mujoco")

    # -------------------------------------------------------------------------
    # Launch 参数
    # -------------------------------------------------------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    use_joy = LaunchConfiguration("use_joy")

    task_file = LaunchConfiguration("task_file")
    xacro_file = LaunchConfiguration("xacro_file")
    urdf_file = LaunchConfiguration("urdf_file")
    lib_folder = LaunchConfiguration("lib_folder")

    declare_args = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "use_joy",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "task_file",
            default_value=PathJoinSubstitution(
                [pkg_ocs2, "config", "task.info"]
            ),
        ),
        DeclareLaunchArgument(
            # 名字沿用你原来的 xacro_file，但现在默认指向 MuJoCo 包里的 .urdf
            "xacro_file",
            default_value=PathJoinSubstitution(
                [pkg_mujoco, "urdf", "tracer_jaka_zu5.urdf"]
            ),
        ),
        DeclareLaunchArgument(
            "urdf_file",
            default_value="/tmp/ocs2_tracer_jaka/tracer_jaka.urdf",
        ),
        DeclareLaunchArgument(
            "lib_folder",
            default_value="/tmp/ocs2_tracer_jaka/auto_generated",
        ),
    ]

    # -------------------------------------------------------------------------
    # Step 1: 准备 OCS2 需要的 URDF 文件
    # -------------------------------------------------------------------------
    prepare_urdf = OpaqueFunction(function=_ensure_urdf)

    # -------------------------------------------------------------------------
    # Step 2: 启动 MuJoCo ros2_control 仿真
    #
    # 这个 launch 里已经包含:
    #   - robot_state_publisher
    #   - mujoco_ros2_control
    #   - joint_state_broadcaster
    #   - arm_controller
    #   - base_controller
    # -------------------------------------------------------------------------
    mujoco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_mujoco, "launch", "bridge.launch.py"]
            )
        ),
            launch_arguments={
                "viewer": "true",
            }.items()
    )

    # -------------------------------------------------------------------------
    # Step 3: OCS2 三件套
    # -------------------------------------------------------------------------
    mpc_node = Node(
        package="tracer_jaka_ocs2",
        executable="tracer_jaka_mpc_node",
        name="tracer_jaka_mpc_node",
        output="screen",
        parameters=[
            {
                "taskFile": task_file,
                "urdfFile": urdf_file,
                "libFolder": lib_folder,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    mrt_node = Node(
        package="tracer_jaka_ocs2",
        executable="tracer_jaka_mrt_node",
        name="tracer_jaka_mrt_node",
        output="screen",
        parameters=[
            {
                "taskFile": task_file,
                "urdfFile": urdf_file,
                "libFolder": lib_folder,

                "mrt_loop_rate": 100.0,
                "traj_horizon": 0.05,
                "traj_num_points": 5,

                "use_stamped_cmd": False,
                
                # 来自 diff_drive_controller
                "odom_topic": "/base_controller/odom",

                # 来自 joint_state_broadcaster
                "joint_state_topic": "/joint_states",

                # 关键修改：
                # controllers.yaml 里 base_controller:
                #   use_stamped_vel: false
                # 所以命令话题是 /base_controller/cmd_vel_unstamped
                "base_cmd_topic": "/base_controller/cmd_vel",

                # JointTrajectoryController 的直接命令话题
                "arm_cmd_topic": "/arm_controller/joint_trajectory",

                "arm_joint_names": [
                    "joint_1",
                    "joint_2",
                    "joint_3",
                    "joint_4",
                    "joint_5",
                    "joint_6",
                ],

                "base_frame": "base_footprint",
                "use_sim_time": use_sim_time,
            }
        ],
    )

    target_node = Node(
        package="tracer_jaka_ocs2",
        executable="tracer_jaka_target_node",
        name="tracer_jaka_target_node",
        output="screen",
        parameters=[
            {
                "robot_name": "mobile_manipulator",
                "marker_frame": "odom",
                "ee_frame": "tool0",
                "marker_scale": 0.3,
                "input_dim": 8,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    # -------------------------------------------------------------------------
    # 可选：手柄驱动
    # -------------------------------------------------------------------------
    joy_driver = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[
            {
                "device_id": 0,
                "deadzone": 0.05,
                "autorepeat_rate": 20.0,
                "use_sim_time": use_sim_time,
            }
        ],
        condition=IfCondition(use_joy),
        output="screen",
    )

    joy_target_node = Node(
        package="tracer_jaka_ocs2",
        executable="tracer_jaka_joy_target_node",
        name="tracer_jaka_joy_target_node",
        output="screen",
        parameters=[
            {
                "robot_name": "mobile_manipulator",
                "marker_frame": "odom",

                # 建议和 target_node 保持一致。
                # 如果你的 URDF 里实际末端 frame 叫 gripper_center_link，
                # 再改回 gripper_center_link。
                "ee_frame": "tool0",

                "input_dim": 8,
                "joy_topic": "/joy",
                "publish_rate": 50.0,

                "linear_speed": 0.15,
                "angular_speed": 0.6,
                "deadzone": 0.10,

                "use_sim_time": use_sim_time,

                # PS4 / PS5 手柄可以在这里覆盖映射
                # "axis_x": 1,
                # "axis_y": 0,
                # "axis_z": 4,
                # "axis_yaw": 3,
                # "button_deadman": 4,
                # "button_reset": 0,
                # "button_home": 1,
            }
        ],
        condition=IfCondition(use_joy),
    )

    # -------------------------------------------------------------------------
    # Step 4: RViz2
    # -------------------------------------------------------------------------
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [pkg_ocs2, "rviz", "tracer_jaka_ocs2.rviz"]
            ),
        ],
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
        condition=IfCondition(use_rviz),
        output="screen",
    )


    map_to_odom_tf = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="map_to_odom_static_tf",
    arguments=[
        "-2", "0", "0",     # x y z
        "0", "0", "0",     # yaw pitch roll
        "map",
        "odom",
    ],
    output="screen",
    )


    # -------------------------------------------------------------------------
    # 启动时序
    #
    # MuJoCo 和 controller_manager 需要一点时间启动；
    # OCS2 MPC codegen / solver 初始化也比较吃时间。
    # -------------------------------------------------------------------------
    rviz_delayed = TimerAction(
        period=4.0,
        actions=[rviz_node],
    )

    ocs2_delayed = TimerAction(
        period=8.0,
        actions=[
            mpc_node,
            mrt_node,
            target_node,
            #joy_driver,
            #joy_target_node,
        ],
    )

    return LaunchDescription(
        declare_args
        + [
            prepare_urdf,
            mujoco_launch,
            rviz_delayed,
            ocs2_delayed,
            map_to_odom_tf,
        ]
    )
