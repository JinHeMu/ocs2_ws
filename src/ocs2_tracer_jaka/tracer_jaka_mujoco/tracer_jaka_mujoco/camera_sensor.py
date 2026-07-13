#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
camera_sensor.py —— MuJoCo 离屏渲染深度/RGB 相机 -> ROS2（独立进程渲染版）

为什么要独立进程？
------------------
线程方案虽然把渲染挪出了物理主循环，但渲染 (mjr_render) 在 GL 调用期间会持有
Python GIL，物理主线程因此拿不到 GIL 去执行 mj_step，导致 sim_time 落后墙钟，
表现为“平滑但整体变慢”。

把渲染放进【独立进程】后：
  * 物理主线程独占 GIL，500Hz 步进保持实时；
  * 渲染进程拥有自己的 EGL 上下文，在另外的核心上跑，渲得快慢都不阻塞物理；
  * 实际帧率 = min(camera.rate, 渲染能力)，sim_time 始终保持实时。

通信：主进程相机线程在 physics_lock 内快照 qpos（极快），通过队列发给渲染进程；
渲染进程 mj_forward + render 后把图像与相机位姿送回，主进程相机线程负责发 ROS。

运行方式（必须无头 + EGL）：
    export MUJOCO_GL=egl
    ros2 launch tracer_jaka_mujoco bridge.launch.py viewer:=false
"""

import os
import time
import queue
import threading
import multiprocessing as mp

import numpy as np
import mujoco

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import TransformBroadcaster

from . import sensors_common as sc


# ====================================================================== #
#  渲染进程入口（在独立进程里运行，拥有自己的 EGL 上下文，不碰 ROS）
# ====================================================================== #
def _render_process(model_path, cam_name, width, height,
                    want_color, want_depth, gl_backend, in_q, out_q):
    # 在创建 GL 上下文前确保后端为 EGL（spawn 已继承父进程环境，这里再兜底）
    if gl_backend:
        os.environ["MUJOCO_GL"] = gl_backend

    import mujoco  # 本进程独立导入与上下文

    try:
        model = mujoco.MjModel.from_xml_path(model_path)
        data = mujoco.MjData(model)
        cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, cam_name)
        if cam_id < 0:
            out_q.put({"fatal": f"渲染进程找不到相机: {cam_name}"})
            return
        renderer = mujoco.Renderer(model, height=height, width=width)
    except Exception as e:                   # noqa
        out_q.put({"fatal": f"渲染进程初始化失败（确认 MUJOCO_GL=egl 与驱动）: {e}"})
        return

    out_q.put({"ready": True})

    while True:
        item = in_q.get()
        if item is None:                     # 收到停止信号
            break

        data.qpos[:] = item["qpos"]
        mujoco.mj_forward(model, data)

        rgb = None
        depth = None
        try:
            renderer.update_scene(data, camera=cam_id)
            if want_color:
                renderer.disable_depth_rendering()
                rgb = renderer.render().copy()
            if want_depth:
                renderer.enable_depth_rendering()
                depth = renderer.render().copy()
                renderer.disable_depth_rendering()
        except Exception as e:               # noqa
            out_q.put({"err": str(e)})
            continue

        cam_pos = data.cam_xpos[cam_id].copy()
        cam_mat = data.cam_xmat[cam_id].reshape(3, 3).copy()
        out_q.put({"rgb": rgb, "depth": depth,
                   "cam_pos": cam_pos, "cam_mat": cam_mat})


# ====================================================================== #
#  主进程侧：相机封装
# ====================================================================== #
class CameraSensor:
    def __init__(self, node, cfg):
        self.node = node
        self.log = node.get_logger()
        self.model = node.model

        self.cam_name = cfg["camera_name"]
        self.width = int(cfg["width"])
        self.height = int(cfg["height"])
        self.fovy = float(cfg["fovy"])
        self.rate = float(cfg["rate"])
        self.color_frame = cfg["color_frame_id"]
        self.depth_frame = cfg["depth_frame_id"]
        self.pub_color = bool(cfg["publish_color"])
        self.pub_depth = bool(cfg["publish_depth"])
        self.pub_info = bool(cfg["publish_camera_info"])
        self.max_depth = float(cfg["max_depth"])
        self.clip_far = bool(cfg["clip_far_to_zero"])

        self._ok = False

        # 主进程只做名字校验（不创建 Renderer，避免在主进程建 GL 上下文）
        self.cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, self.cam_name)
        if self.cam_id < 0:
            self.log.error(f"[camera] 模型中找不到相机: {self.cam_name}，相机已禁用")
            return

        # ---- ROS 接口 ----
        if self.pub_color:
            self.color_pub = node.create_publisher(
                Image, cfg["color_topic"], qos_profile_sensor_data)
            self.color_info_pub = node.create_publisher(
                CameraInfo, cfg["color_info_topic"], qos_profile_sensor_data)
        if self.pub_depth:
            self.depth_pub = node.create_publisher(
                Image, cfg["depth_topic"], qos_profile_sensor_data)
            self.depth_info_pub = node.create_publisher(
                CameraInfo, cfg["depth_info_topic"], qos_profile_sensor_data)
        self.tf_bc = TransformBroadcaster(node)

        # ---- 进程 / 线程句柄 ----
        self._proc = None
        self._in_q = None
        self._out_q = None
        self._thread = None
        self._running = False

        self._ok = True
        self.log.info(
            f"[camera] enabled  {self.width}x{self.height}  rate={self.rate}Hz "
            f"cam={self.cam_name}（渲染在独立进程）")

    # ---------------------------------------------------------------- #
    @property
    def ok(self):
        return self._ok

    # ---------------------------------------------------------------- #
    def start(self):
        if not self._ok:
            return
        if getattr(self.node, "use_viewer", False):
            self.log.warn(
                "[camera] use_viewer=true 时不建议开相机；本方案要求 "
                "viewer:=false 且 export MUJOCO_GL=egl")

        gl_backend = os.environ.get("MUJOCO_GL", "egl")

        # spawn：避免 fork 继承 GL/CUDA 上下文导致的崩溃
        ctx = mp.get_context("spawn")
        self._in_q = ctx.Queue(maxsize=1)
        self._out_q = ctx.Queue(maxsize=2)
        self._proc = ctx.Process(
            target=_render_process,
            args=(self.node.model_path, self.cam_name, self.width, self.height,
                  self.pub_color, self.pub_depth, gl_backend,
                  self._in_q, self._out_q),
            daemon=True,
        )
        self._proc.start()

        # 等待渲染进程握手（成功 / 致命错误）
        try:
            hello = self._out_q.get(timeout=20.0)
        except queue.Empty:
            self.log.error("[camera] 渲染进程启动超时，相机禁用")
            self._ok = False
            return
        if "fatal" in hello:
            self.log.error(f"[camera] {hello['fatal']}，相机禁用")
            self._ok = False
            return

        self._running = True
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._in_q is not None:
            try:
                self._in_q.put_nowait(None)     # 通知渲染进程退出
            except Exception:                   # noqa
                pass
        if self._proc is not None:
            self._proc.join(timeout=2.0)
            if self._proc.is_alive():
                self._proc.terminate()

    # ---------------------------------------------------------------- #
    def _snapshot(self):
        """物理锁内快照 qpos（极快），并取底盘位姿。"""
        with self.node.physics_lock:
            qpos = self.node.data.qpos.copy()
            base = (float(self.node.base_x),
                    float(self.node.base_y),
                    float(self.node.base_yaw))
        return qpos, base

    # ---------------------------------------------------------------- #
    def _publish_tf(self, stamp, cam_pos, cam_mat, base):
        x, y, yaw = base
        R_w_b, t_w_b = sc.base_world_pose(x, y, yaw)
        R_w_opt = cam_mat @ sc.MUJOCO_CAM_TO_OPTICAL          # MuJoCo 相机系 -> ROS 光学系
        R_b_opt, t_b_opt = sc.world_to_base(R_w_b, t_w_b, R_w_opt, cam_pos)
        if self.pub_depth:
            self.tf_bc.sendTransform(
                sc.make_tf(stamp, self.node.base_frame, self.depth_frame, R_b_opt, t_b_opt))
        if self.pub_color and self.color_frame != self.depth_frame:
            self.tf_bc.sendTransform(
                sc.make_tf(stamp, self.node.base_frame, self.color_frame, R_b_opt, t_b_opt))

    # ---------------------------------------------------------------- #
    def _publish(self, stamp, base, res):
        rgb = res.get("rgb")
        depth = res.get("depth")

        if self.pub_color and rgb is not None:
            self.color_pub.publish(sc.make_image_rgb8(stamp, self.color_frame, rgb))
            if self.pub_info:
                self.color_info_pub.publish(
                    sc.make_camera_info(stamp, self.color_frame,
                                        self.width, self.height, self.fovy))

        if self.pub_depth and depth is not None:
            depth = np.asarray(depth, dtype=np.float32)
            depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
            if self.clip_far:
                depth[depth > self.max_depth] = 0.0
            self.depth_pub.publish(sc.make_image_depth32(stamp, self.depth_frame, depth))
            if self.pub_info:
                self.depth_info_pub.publish(
                    sc.make_camera_info(stamp, self.depth_frame,
                                        self.width, self.height, self.fovy))

        self._publish_tf(stamp, res["cam_pos"], res["cam_mat"], base)

    # ---------------------------------------------------------------- #
    def _worker(self):
        """
        在【相机线程】里与渲染进程做同步往返：
        发送 qpos -> 阻塞等待渲染结果 -> 发布。
        等待期间该线程释放 GIL，物理主线程不受影响。
        """
        period = 1.0 / self.rate
        next_t = time.perf_counter()

        while self._running:
            qpos, base = self._snapshot()
            stamp = self.node.now_msg()

            res = None
            try:
                self._in_q.put({"qpos": qpos}, timeout=1.0)
                res = self._out_q.get(timeout=5.0)
            except queue.Full:
                self.log.warn("[camera] 渲染进程繁忙，丢帧", throttle_duration_sec=5.0)
            except queue.Empty:
                self.log.warn("[camera] 等待渲染结果超时", throttle_duration_sec=5.0)

            if res is not None:
                if "err" in res:
                    self.log.warn(f"[camera] 渲染失败: {res['err']}",
                                  throttle_duration_sec=5.0)
                elif "fatal" in res:
                    self.log.error(f"[camera] {res['fatal']}，相机线程退出")
                    break
                else:
                    self._publish(stamp, base, res)

            # 按 camera.rate 节流（往返本身慢于 period 时自动退化为“渲多快发多快”）
            next_t += period
            sleep = next_t - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.perf_counter()
