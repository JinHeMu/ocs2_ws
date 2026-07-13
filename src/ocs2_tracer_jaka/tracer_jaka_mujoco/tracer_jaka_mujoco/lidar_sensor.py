#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lidar_sensor.py —— MuJoCo-LiDAR 封装为 ROS2 传感器

要点
----
* 在 **独立线程** 中按 lidar.rate 跑光线追踪，不阻塞 500Hz 物理步进。
* 每次扫描前在锁内把 MjData 快照到私有副本，再在锁外做耗时的 trace_rays，
  既保证数据一致，又不长时间占用物理锁。
* 点云发布在雷达自身坐标系（get_hit_points 返回的就是局部点），
  并发布 base_footprint -> lidar_frame 的静态 TF。
* 扫描模式、分辨率、频率、cutoff、backend 全部来自配置文件。
"""

import math
import time
import threading

import numpy as np
import mujoco

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from tf2_ros import StaticTransformBroadcaster

from . import sensors_common as sc


class LidarSensor:
    def __init__(self, node, cfg):
        self.node = node
        self.log = node.get_logger()
        self.model = node.model

        self.site_name = cfg["site_name"]
        self.frame_id = cfg["frame_id"]
        self.topic = cfg["topic"]
        self.rate = float(cfg["rate"])
        self.backend = cfg["backend"]
        self.cutoff = float(cfg["cutoff_dist"])
        self.pattern = cfg["pattern"]
        self.cols = int(cfg["num_ray_cols"])
        self.rows = int(cfg["num_ray_rows"])
        self.device_mem_gb = float(cfg["taichi_device_memory_gb"])

        self._ok = False
        self._thread = None
        self._running = False

        # ---- 校验 site ----
        sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, self.site_name)
        if sid < 0:
            self.log.error(f"[lidar] 模型中找不到 site: {self.site_name}，LiDAR 已禁用")
            return

        # ---- 导入并创建 MuJoCo-LiDAR ----
        try:
            from mujoco_lidar import MjLidarWrapper, scan_gen
        except Exception as e:               # noqa
            self.log.error(f"[lidar] 无法导入 mujoco_lidar（pip install mujoco_lidar）: {e}")
            return

        self._scan_gen = scan_gen
        try:
            self.theta, self.phi = self._build_pattern(scan_gen)
        except Exception as e:               # noqa
            self.log.error(f"[lidar] 生成扫描模式失败: {e}")
            return

        args = {}
        if self.backend == "taichi":
            args = {"ti_init_args": {"device_memory_GB": self.device_mem_gb}}
        try:
            self.lidar = MjLidarWrapper(
                self.model,
                site_name=self.site_name,
                backend=self.backend,
                cutoff_dist=self.cutoff,
                args=args,
            )
        except Exception as e:               # noqa
            self.log.error(f"[lidar] 创建 MjLidarWrapper 失败: {e}")
            return

        # ---- 私有数据快照 ----
        self._snap = mujoco.MjData(self.model)

        # ---- ROS 接口 ----
        self.pub = node.create_publisher(PointCloud2, self.topic, qos_profile_sensor_data)
        self.static_tf = StaticTransformBroadcaster(node)
        self._tf_sent = False

        self._ok = True
        self.log.info(
            f"[lidar] enabled  rays={len(self.theta)}  rate={self.rate}Hz  "
            f"backend={self.backend}  topic={self.topic}")

    # ---------------------------------------------------------------- #
    def _build_pattern(self, scan_gen):
        """根据配置返回 (theta, phi)。pattern=grid 用 cols/rows，其余尝试命名模式。"""
        if self.pattern == "grid":
            return scan_gen.generate_grid_scan_pattern(
                num_ray_cols=self.cols, num_ray_rows=self.rows)
        fn = getattr(scan_gen, f"generate_{self.pattern}", None)
        if fn is None:
            self.log.warn(
                f"[lidar] 未知扫描模式 '{self.pattern}'，回退到 grid {self.cols}x{self.rows}")
            return scan_gen.generate_grid_scan_pattern(
                num_ray_cols=self.cols, num_ray_rows=self.rows)
        return fn()

    # ---------------------------------------------------------------- #
    def start(self):
        if not self._ok:
            return
        self._running = True
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False

    # ---------------------------------------------------------------- #
    def _snapshot(self):
        """在物理锁内把 data 拷到私有副本（快），锁外再做光追。"""
        with self.node.physics_lock:
            try:
                mujoco.mj_copyData(self._snap, self.model, self.node.data)
            except Exception:                # noqa  老版本兜底
                self._snap.qpos[:] = self.node.data.qpos
                self._snap.qvel[:] = self.node.data.qvel
                mujoco.mj_forward(self.model, self._snap)

    def _publish_static_tf(self, stamp):
        """雷达固连底盘 -> 静态 TF，仅发一次。"""
        site = self._snap.site(self.site_name)
        R_w_s = site.xmat.reshape(3, 3).copy()
        t_w_s = site.xpos.copy()
        x = float(self.node.base_x)
        y = float(self.node.base_y)
        yaw = float(self.node.base_yaw)
        R_w_b, t_w_b = sc.base_world_pose(x, y, yaw)
        R_b_s, t_b_s = sc.world_to_base(R_w_b, t_w_b, R_w_s, t_w_s)
        tf = sc.make_tf(stamp, self.node.base_frame, self.frame_id, R_b_s, t_b_s)
        self.static_tf.sendTransform(tf)
        self._tf_sent = True

    def _worker(self):
        period = 1.0 / self.rate
        next_t = time.perf_counter()
        while self._running:
            self._snapshot()
            stamp = self.node.now_msg()
            try:
                ranges = self.lidar.trace_rays(self._snap, self.theta, self.phi)
                pts = np.asarray(self.lidar.get_hit_points(), dtype=np.float32).reshape(-1, 3)
            except Exception as e:           # noqa
                self.log.warn(f"[lidar] trace_rays 失败: {e}", throttle_duration_sec=5.0)
                pts = np.empty((0, 3), dtype=np.float32)
                ranges = None

            # 过滤无效命中
            if ranges is not None and len(ranges) == len(pts):
                mask = np.asarray(ranges) > 1e-4
                pts = pts[mask]
            finite = np.isfinite(pts).all(axis=1)
            pts = pts[finite]

            if not self._tf_sent:
                self._publish_static_tf(stamp)

            self.pub.publish(sc.make_pointcloud2(stamp, self.frame_id, pts))

            next_t += period
            sleep = next_t - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.perf_counter()
