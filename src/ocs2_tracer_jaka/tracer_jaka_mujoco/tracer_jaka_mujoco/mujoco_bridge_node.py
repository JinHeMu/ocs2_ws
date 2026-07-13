#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mujoco_bridge_node.py —— ROS2 <-> MuJoCo 自包含桥接节点（含 LiDAR + 深度相机）

在原“纯桥接（3 自由度平面底盘 + 6 轴机械臂）”基础上新增：
  * LiDAR：mujoco_lidar 光线追踪，独立线程，频率/分辨率/扫描模式可配
  * 深度/RGB 相机：MuJoCo 离屏渲染，主线程抽帧渲染 + 独立线程发布，帧率可配
  * 传感器频率与物理步进(默认 500Hz)解耦，避免“传感器跟着物理跑导致卡顿”

时间源 / 执行者：本节点加载 MuJoCo、步进物理、发布 /clock。
所有参数（含传感器）均可由 config/sensors.yaml 提供。
"""

import math
import threading
import time

import numpy as np
import mujoco
import mujoco.viewer

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from builtin_interfaces.msg import Time as TimeMsg
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory
from control_msgs.action import FollowJointTrajectory
from tf2_ros import TransformBroadcaster

from .lidar_sensor import LidarSensor
from .camera_sensor import CameraSensor


ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
WHEELS = ["left_wheel", "right_wheel"]


def yaw_to_quat(yaw):
    return math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)


def secs_to_time_msg(t):
    sec = int(t)
    nsec = int(round((t - sec) * 1e9))
    if nsec >= 1_000_000_000:
        sec += 1
        nsec -= 1_000_000_000
    m = TimeMsg()
    m.sec = sec
    m.nanosec = nsec
    return m


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


class MujocoBridge(Node):
    def __init__(self):
        super().__init__("mujoco_bridge")

        # ---------------- 基本参数 ----------------
        self.declare_parameter("model_path", "")
        self.declare_parameter("base_x_joint", "base_x")
        self.declare_parameter("base_y_joint", "base_y")
        self.declare_parameter("base_yaw_joint", "base_yaw")
        self.declare_parameter("wheel_separation", 0.34)
        self.declare_parameter("wheel_radius", 0.065)
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("clock_rate", 250.0)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_odom_tf", True)
        self.declare_parameter("exact_base", True)
        self.declare_parameter("spin_wheels", True)
        self.declare_parameter("use_viewer", True)
        self.declare_parameter("arm_actuator_suffix", "_servo")
        self.declare_parameter("init_keyframe", "home")

        # ---------------- LiDAR 参数 ----------------
        self.declare_parameter("lidar.enable", False)
        self.declare_parameter("lidar.site_name", "lidar_site")
        self.declare_parameter("lidar.frame_id", "lidar_link")
        self.declare_parameter("lidar.topic", "/lidar/points")
        self.declare_parameter("lidar.rate", 25.0)
        self.declare_parameter("lidar.backend", "cpu")          # cpu | taichi
        self.declare_parameter("lidar.cutoff_dist", 5.0)
        self.declare_parameter("lidar.pattern", "grid")         # grid | HDL64 | ...
        self.declare_parameter("lidar.num_ray_cols", 64)
        self.declare_parameter("lidar.num_ray_rows", 16)
        self.declare_parameter("lidar.taichi_device_memory_gb", 2.0)

        # ---------------- 相机参数 ----------------
        self.declare_parameter("camera.enable", False)
        self.declare_parameter("camera.camera_name", "d435_depth")
        self.declare_parameter("camera.width", 640)
        self.declare_parameter("camera.height", 480)
        self.declare_parameter("camera.fovy", 58.0)
        self.declare_parameter("camera.rate", 15.0)
        self.declare_parameter("camera.color_frame_id", "camera_color_optical_frame")
        self.declare_parameter("camera.depth_frame_id", "camera_depth_optical_frame")
        self.declare_parameter("camera.color_topic", "/camera/color/image_raw")
        self.declare_parameter("camera.color_info_topic", "/camera/color/camera_info")
        self.declare_parameter("camera.depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("camera.depth_info_topic", "/camera/depth/camera_info")
        self.declare_parameter("camera.publish_color", True)
        self.declare_parameter("camera.publish_depth", True)
        self.declare_parameter("camera.publish_camera_info", True)
        self.declare_parameter("camera.max_depth", 5.0)
        self.declare_parameter("camera.clip_far_to_zero", True)

        gp = lambda n: self.get_parameter(n).value
        self.model_path = gp("model_path")
        self.bx_name = gp("base_x_joint")
        self.by_name = gp("base_y_joint")
        self.byaw_name = gp("base_yaw_joint")
        self.wheel_sep = float(gp("wheel_separation"))
        self.wheel_rad = float(gp("wheel_radius"))
        self.publish_rate = float(gp("publish_rate"))
        self.clock_rate = float(gp("clock_rate"))
        self.odom_frame = gp("odom_frame")
        self.base_frame = gp("base_frame")
        self.publish_odom_tf = bool(gp("publish_odom_tf"))
        self.exact_base = bool(gp("exact_base"))
        self.spin_wheels = bool(gp("spin_wheels"))
        self.use_viewer = bool(gp("use_viewer"))
        arm_suffix = gp("arm_actuator_suffix")
        init_key = gp("init_keyframe")

        if not self.model_path:
            raise RuntimeError("必须通过参数 model_path 指定 MuJoCo XML 路径（建议指向 scene.xml）")

        # ---------------- 加载模型 ----------------
        self.model = mujoco.MjModel.from_xml_path(self.model_path)
        self.data = mujoco.MjData(self.model)
        self.dt = self.model.opt.timestep

        key_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, init_key)
        if key_id < 0:
            raise RuntimeError(f"找不到 keyframe: {init_key}")
        mujoco.mj_resetDataKeyframe(self.model, self.data, key_id)
        mujoco.mj_forward(self.model, self.data)

        nid = lambda t, n: mujoco.mj_name2id(self.model, t, n)

        self.qadr, self.dadr = {}, {}
        for j in ARM_JOINTS + WHEELS + [self.bx_name, self.by_name, self.byaw_name]:
            i = nid(mujoco.mjtObj.mjOBJ_JOINT, j)
            if i < 0:
                self.get_logger().warn(f"模型中找不到关节 {j}")
                continue
            self.qadr[j] = self.model.jnt_qposadr[i]
            self.dadr[j] = self.model.jnt_dofadr[i]

        for j in (self.bx_name, self.by_name, self.byaw_name):
            if j not in self.qadr:
                raise RuntimeError(f"找不到底盘关节: {j}")

        self.a_arm = [nid(mujoco.mjtObj.mjOBJ_ACTUATOR, f"{j}{arm_suffix}")
                      for j in ARM_JOINTS]

        # ---------------- 状态 / 锁 ----------------
        self.lock = threading.Lock()          # 保护 cmd / arm 目标
        self.physics_lock = threading.Lock()  # 保护 mj_step 与传感器快照
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.arm_target = np.array(
            [self.data.qpos[self.qadr[j]] for j in ARM_JOINTS], dtype=float)
        self.arm_traj = None
        self.traj_start = 0.0

        self.base_x = float(self.data.qpos[self.qadr[self.bx_name]])
        self.base_y = float(self.data.qpos[self.qadr[self.by_name]])
        self.base_yaw = float(self.data.qpos[self.qadr[self.byaw_name]])
        self.wheel_pos = {w: (float(self.data.qpos[self.qadr[w]]) if w in self.qadr else 0.0)
                          for w in WHEELS}
        self.sim_time = 0.0

        # ---------------- ROS 接口 ----------------
        cb = ReentrantCallbackGroup()
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10, callback_group=cb)
        self.create_subscription(Twist, "/base_controller/cmd_vel", self.on_cmd_vel, 10,
                                 callback_group=cb)
        self.create_subscription(JointTrajectory, "/arm_controller/joint_trajectory",
                                 self.on_arm_traj, 10, callback_group=cb)
        self.create_subscription(Float64MultiArray, "/arm_controller/commands",
                                 self.on_arm_forward, 10, callback_group=cb)
        self.create_subscription(Float64MultiArray, "/arm_command",
                                 self.on_arm_forward, 10, callback_group=cb)

        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.odom_pub = self.create_publisher(Odometry, "/base_controller/odom", 10)
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.tf_bc = TransformBroadcaster(self)

        self._action = ActionServer(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory",
            execute_callback=self.execute_traj_action,
            goal_callback=lambda g: GoalResponse.ACCEPT,
            cancel_callback=lambda g: CancelResponse.ACCEPT,
            callback_group=cb)

        # ---------------- 传感器 ----------------
        self.lidar = None
        self.camera = None
        if bool(gp("lidar.enable")):
            self.lidar = LidarSensor(self, {
                "site_name": gp("lidar.site_name"),
                "frame_id": gp("lidar.frame_id"),
                "topic": gp("lidar.topic"),
                "rate": gp("lidar.rate"),
                "backend": gp("lidar.backend"),
                "cutoff_dist": gp("lidar.cutoff_dist"),
                "pattern": gp("lidar.pattern"),
                "num_ray_cols": gp("lidar.num_ray_cols"),
                "num_ray_rows": gp("lidar.num_ray_rows"),
                "taichi_device_memory_gb": gp("lidar.taichi_device_memory_gb"),
            })
        if bool(gp("camera.enable")):
            self.camera = CameraSensor(self, {
                "camera_name": gp("camera.camera_name"),
                "width": gp("camera.width"),
                "height": gp("camera.height"),
                "fovy": gp("camera.fovy"),
                "rate": gp("camera.rate"),
                "color_frame_id": gp("camera.color_frame_id"),
                "depth_frame_id": gp("camera.depth_frame_id"),
                "color_topic": gp("camera.color_topic"),
                "color_info_topic": gp("camera.color_info_topic"),
                "depth_topic": gp("camera.depth_topic"),
                "depth_info_topic": gp("camera.depth_info_topic"),
                "publish_color": gp("camera.publish_color"),
                "publish_depth": gp("camera.publish_depth"),
                "publish_camera_info": gp("camera.publish_camera_info"),
                "max_depth": gp("camera.max_depth"),
                "clip_far_to_zero": gp("camera.clip_far_to_zero"),
            })

        self.get_logger().info(
            f"Loaded {self.model_path}  dt={self.dt:.4f}  exact_base={self.exact_base}")

    # ---------- 给传感器线程用的统一时间戳 ----------
    def now_msg(self):
        return secs_to_time_msg(self.sim_time)

    def start_sensors(self):
        if self.lidar:
            self.lidar.start()
        if self.camera:
            self.camera.start()

    def stop_sensors(self):
        if self.lidar:
            self.lidar.stop()
        if self.camera:
            self.camera.stop()

    # ============================================================
    #  订阅回调
    # ============================================================
    def on_cmd_vel(self, msg: Twist):
        with self.lock:
            self.cmd_v = float(msg.linear.x)
            self.cmd_w = float(msg.angular.z)

    def on_arm_forward(self, msg: Float64MultiArray):
        if len(msg.data) >= 6:
            with self.lock:
                self.arm_traj = None
                self.arm_target = np.array(msg.data[:6], dtype=float)

    def _parse_trajectory(self, traj: JointTrajectory):
        if not traj.points:
            return None
        idx = {name: i for i, name in enumerate(traj.joint_names)}
        cols = [idx.get(j, None) for j in ARM_JOINTS]
        out = []
        for p in traj.points:
            q = self.arm_target.copy()
            for k, c in enumerate(cols):
                if c is not None and c < len(p.positions):
                    q[k] = float(p.positions[c])
            t = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
            out.append((t, q))
        out.sort(key=lambda x: x[0])
        return out

    def on_arm_traj(self, msg: JointTrajectory):
        parsed = self._parse_trajectory(msg)
        if parsed is None:
            return
        with self.lock:
            self.arm_traj = parsed
            self.traj_start = self.sim_time

    # ============================================================
    #  FollowJointTrajectory action（MoveIt）
    # ============================================================
    def execute_traj_action(self, goal_handle):
        traj = goal_handle.request.trajectory
        parsed = self._parse_trajectory(traj)
        result = FollowJointTrajectory.Result()
        if parsed is None:
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result

        with self.lock:
            self.arm_traj = parsed
            self.traj_start = self.sim_time
        duration = parsed[-1][0]

        while rclpy.ok():
            with self.lock:
                elapsed = self.sim_time - self.traj_start
                if self.arm_traj is not parsed:
                    break
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result
            if elapsed >= duration:
                break
            time.sleep(0.02)

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    # ============================================================
    #  控制 / 物理步进
    # ============================================================
    def _interp_arm(self):
        traj = self.arm_traj
        if traj is None:
            return self.arm_target
        elapsed = self.sim_time - self.traj_start
        if elapsed <= traj[0][0]:
            return traj[0][1]
        if elapsed >= traj[-1][0]:
            return traj[-1][1]
        for i in range(1, len(traj)):
            t0, q0 = traj[i - 1]
            t1, q1 = traj[i]
            if elapsed <= t1:
                a = (elapsed - t0) / max(1e-6, (t1 - t0))
                return q0 + a * (q1 - q0)
        return traj[-1][1]

    def _apply_arm_ctrl(self):
        with self.lock:
            self.arm_target = self._interp_arm()
            arm = self.arm_target.copy()
        for k, aid in enumerate(self.a_arm):
            if aid >= 0:
                self.data.ctrl[aid] = arm[k]

    def _integrate_planar_base(self):
        with self.lock:
            v, w = self.cmd_v, self.cmd_w
        dt = self.dt
        yaw_mid = self.base_yaw + 0.5 * w * dt
        self.base_x += v * math.cos(yaw_mid) * dt
        self.base_y += v * math.sin(yaw_mid) * dt
        self.base_yaw = wrap_pi(self.base_yaw + w * dt)

        self.data.qpos[self.qadr[self.bx_name]] = self.base_x
        self.data.qpos[self.qadr[self.by_name]] = self.base_y
        self.data.qpos[self.qadr[self.byaw_name]] = self.base_yaw
        self.data.qvel[self.dadr[self.bx_name]] = v * math.cos(self.base_yaw)
        self.data.qvel[self.dadr[self.by_name]] = v * math.sin(self.base_yaw)
        self.data.qvel[self.dadr[self.byaw_name]] = w

        if self.spin_wheels:
            wl = (v - w * self.wheel_sep / 2.0) / self.wheel_rad
            wr = (v + w * self.wheel_sep / 2.0) / self.wheel_rad
            for name, omega in (("left_wheel", wl), ("right_wheel", wr)):
                if name in self.qadr:
                    self.wheel_pos[name] += omega * dt
                    self.data.qpos[self.qadr[name]] = self.wheel_pos[name]
                    self.data.qvel[self.dadr[name]] = omega

    def step(self):
        """单步物理。整段在 physics_lock 内，保证传感器快照拿到一致状态。"""
        with self.physics_lock:
            self._apply_arm_ctrl()
            mujoco.mj_step(self.model, self.data)
            if self.exact_base:
                self._integrate_planar_base()
                mujoco.mj_forward(self.model, self.data)
        self.sim_time += self.dt

    # ============================================================
    #  发布
    # ============================================================
    def publish_clock(self):
        c = Clock()
        c.clock = secs_to_time_msg(self.sim_time)
        self.clock_pub.publish(c)

    def publish_state(self):
        now = secs_to_time_msg(self.sim_time)

        js = JointState()
        js.header.stamp = now
        names = [j for j in ARM_JOINTS + WHEELS if j in self.qadr]
        js.name = names
        js.position = [float(self.data.qpos[self.qadr[j]]) for j in names]
        js.velocity = [float(self.data.qvel[self.dadr[j]]) for j in names]
        self.js_pub.publish(js)

        x = float(self.data.qpos[self.qadr[self.bx_name]])
        y = float(self.data.qpos[self.qadr[self.by_name]])
        yaw = float(self.data.qpos[self.qadr[self.byaw_name]])
        vx_w = float(self.data.qvel[self.dadr[self.bx_name]])
        vy_w = float(self.data.qvel[self.dadr[self.by_name]])
        wz = float(self.data.qvel[self.dadr[self.byaw_name]])
        c, s = math.cos(yaw), math.sin(yaw)
        v_body = vx_w * c + vy_w * s
        vy_body = -vx_w * s + vy_w * c
        oq_w, _, _, oq_z = yaw_to_quat(yaw)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = oq_z
        odom.pose.pose.orientation.w = oq_w
        odom.twist.twist.linear.x = v_body
        odom.twist.twist.linear.y = vy_body
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

        if self.publish_odom_tf:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation.z = oq_z
            t.transform.rotation.w = oq_w
            self.tf_bc.sendTransform(t)


def _run_loop(node, with_viewer):
    pub_decim = max(1, int(round((1.0 / node.publish_rate) / node.dt)))
    clk_decim = max(1, int(round((1.0 / node.clock_rate) / node.dt)))

    def body(viewer=None):
        step = 0
        next_wall = time.perf_counter()
        while rclpy.ok() and (viewer is None or viewer.is_running()):
            node.step()

            if step % clk_decim == 0:
                node.publish_clock()
            if step % pub_decim == 0:
                node.publish_state()
            # 相机渲染已移至相机自身的工作线程（独立 EGL 上下文），主循环不再渲染
            if viewer is not None:
                viewer.sync()

            step += 1
            next_wall += node.dt
            sleep = next_wall - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_wall = time.perf_counter()

    if with_viewer:
        with mujoco.viewer.launch_passive(
            node.model, node.data,
            show_left_ui=False, show_right_ui=False,
        ) as viewer:
            node.start_sensors()
            body(viewer)
    else:
        node.start_sensors()
        body(None)



def main():
    rclpy.init()
    node = MujocoBridge()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    try:
        _run_loop(node, node.use_viewer)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_sensors()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
