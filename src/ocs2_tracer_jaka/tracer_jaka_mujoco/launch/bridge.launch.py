#!/usr/bin/env python3
"""
bridge.launch.py —— 纯桥接 + LiDAR + 深度相机

启动:
  ros2 launch tracer_jaka_mujoco bridge.launch.py
  ros2 launch tracer_jaka_mujoco bridge.launch.py viewer:=false        # 无头(配合 MUJOCO_GL=egl 高帧率)
  ros2 launch tracer_jaka_mujoco bridge.launch.py model:=/abs/path/scene.xml

控制:
  底盘:  ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear:{x:0.2}, angular:{z:0.3}}"
  机械臂(forward): ros2 topic pub /arm_controller/commands std_msgs/Float64MultiArray "{data:[0,0.5,1.0,0,0.5,0]}"
  机械臂(MoveIt):  action /arm_controller/follow_joint_trajectory

传感器话题:
  /lidar/points                       (sensor_msgs/PointCloud2, 帧 lidar_link)
  /camera/color/image_raw             (sensor_msgs/Image rgb8)
  /camera/depth/image_raw             (sensor_msgs/Image 32FC1, 单位 m)
  /camera/{color,depth}/camera_info   (sensor_msgs/CameraInfo)

要点:
  * 桥接节点自身 use_sim_time=false（它是 /clock 的产生者）。
  * 传感器频率在 config/sensors.yaml 里调（lidar.rate / camera.rate），与 500Hz 物理解耦。
  * model 必须指向带墙体/物体的 scene.xml，否则 LiDAR 没东西可打。
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = "tracer_jaka_mujoco"
    share = get_package_share_directory(pkg)

    default_model = os.path.join(share, "models", "scene.xml")
    sensors_yaml = os.path.join(share, "config", "sensors.yaml")
    urdf = os.path.join(share, "urdf", "tracer_jaka_zu5.urdf")

    model = LaunchConfiguration("model")
    use_viewer = LaunchConfiguration("viewer")

    nodes = [
        DeclareLaunchArgument("model", default_value=default_model,
                              description="MuJoCo 场景 XML（建议 scene.xml）"),
        DeclareLaunchArgument("viewer", default_value="true",
                              description="是否打开 MuJoCo 自带可视化窗口"),

        Node(
            package=pkg, executable="mujoco_bridge", name="mujoco_bridge",
            output="screen",
            parameters=[
                sensors_yaml,                       # 全部传感器/底盘参数
                {                                    # 覆盖：运行期决定的值
                    "model_path": model,
                    "use_sim_time": False,
                    "use_viewer": ParameterValue(
                        PythonExpression(["'", use_viewer, "'.lower() == 'true'"]),
                        value_type=bool),
                },
            ],
        ),
    ]

    # 有 URDF 才启动 robot_state_publisher（提供机械臂各 link 的 TF；传感器不依赖它）
    if os.path.isfile(urdf):
        with open(urdf, "r") as f:
            robot_description = {"robot_description": f.read()}
        nodes.append(
            Node(package="robot_state_publisher", executable="robot_state_publisher",
                 output="screen",
                 parameters=[robot_description, {"use_sim_time": True}]))

    return LaunchDescription(nodes)
