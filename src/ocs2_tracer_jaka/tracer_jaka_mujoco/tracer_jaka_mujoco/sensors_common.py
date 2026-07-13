#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sensors_common.py —— 传感器公用工具

集中放置：
  * 旋转矩阵 <-> 四元数
  * 由 (x, y, yaw) 构造 base_footprint 的世界位姿
  * 把 “世界系传感器位姿” 换算成 “相对 base_footprint 的位姿”
  * PointCloud2 / Image / CameraInfo 的快速构造（不依赖 cv_bridge / pcl）
"""

import math
import numpy as np

from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo
from geometry_msgs.msg import TransformStamped


# ------------------------------------------------------------------ #
#  数学
# ------------------------------------------------------------------ #
def rotmat_to_quat(R):
    """3x3 旋转矩阵 -> (x, y, z, w)。R 为 numpy (3,3)。"""
    R = np.asarray(R, dtype=float).reshape(3, 3)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return x / n, y / n, z / n, w / n


def base_world_pose(x, y, yaw):
    """base_footprint 在世界系下的 (R, t)。底盘只在平面内运动。"""
    c, s = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -s, 0.0],
                  [s,  c, 0.0],
                  [0.0, 0.0, 1.0]])
    t = np.array([x, y, 0.0])
    return R, t


def world_to_base(R_w_base, t_w_base, R_w_s, t_w_s):
    """把世界系传感器位姿 (R_w_s, t_w_s) 换算到 base_footprint 系。"""
    R_bw = R_w_base.T
    R_b_s = R_bw @ R_w_s
    t_b_s = R_bw @ (t_w_s - t_w_base)
    return R_b_s, t_b_s


# MuJoCo 相机系(x右, y上, z朝后/视线为 -z) -> ROS 光学系(x右, y下, z朝前)
# 等价于绕 x 轴转 180°： diag(1, -1, -1)
MUJOCO_CAM_TO_OPTICAL = np.diag([1.0, -1.0, -1.0])


# ------------------------------------------------------------------ #
#  TF
# ------------------------------------------------------------------ #
def make_tf(stamp, parent, child, R, t):
    tf = TransformStamped()
    tf.header.stamp = stamp
    tf.header.frame_id = parent
    tf.child_frame_id = child
    tf.transform.translation.x = float(t[0])
    tf.transform.translation.y = float(t[1])
    tf.transform.translation.z = float(t[2])
    qx, qy, qz, qw = rotmat_to_quat(R)
    tf.transform.rotation.x = qx
    tf.transform.rotation.y = qy
    tf.transform.rotation.z = qz
    tf.transform.rotation.w = qw
    return tf


# ------------------------------------------------------------------ #
#  消息构造
# ------------------------------------------------------------------ #
def make_pointcloud2(stamp, frame_id, points):
    """points: (N,3) float -> sensor_msgs/PointCloud2 (xyz, float32)。"""
    pts = np.asarray(points, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] != 3:
        pts = pts.reshape(-1, 3).astype(np.float32)

    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = pts.shape[0]
    msg.fields = [
        PointField(name="x", offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * pts.shape[0]
    msg.is_dense = True
    msg.data = pts.tobytes()
    return msg


def make_image_rgb8(stamp, frame_id, rgb):
    """rgb: (H,W,3) uint8 -> sensor_msgs/Image (rgb8)。"""
    h, w = rgb.shape[0], rgb.shape[1]
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = h
    msg.width = w
    msg.encoding = "rgb8"
    msg.is_bigendian = 0
    msg.step = w * 3
    msg.data = np.ascontiguousarray(rgb, dtype=np.uint8).tobytes()
    return msg


def make_image_depth32(stamp, frame_id, depth_m):
    """depth_m: (H,W) float, 单位 m -> sensor_msgs/Image (32FC1)。"""
    h, w = depth_m.shape[0], depth_m.shape[1]
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = h
    msg.width = w
    msg.encoding = "32FC1"
    msg.is_bigendian = 0
    msg.step = w * 4
    msg.data = np.ascontiguousarray(depth_m, dtype=np.float32).tobytes()
    return msg


def make_camera_info(stamp, frame_id, width, height, fovy_deg):
    """由 MuJoCo 的垂直 FOV(fovy, 度) 构造针孔模型 CameraInfo。"""
    fovy = math.radians(fovy_deg)
    fy = 0.5 * height / math.tan(0.5 * fovy)
    fx = fy                      # 方形像素
    cx = width / 2.0
    cy = height / 2.0

    info = CameraInfo()
    info.header.stamp = stamp
    info.header.frame_id = frame_id
    info.width = width
    info.height = height
    info.distortion_model = "plumb_bob"
    info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    info.k = [fx, 0.0, cx,
              0.0, fy, cy,
              0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0,
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0]
    info.p = [fx, 0.0, cx, 0.0,
              0.0, fy, cy, 0.0,
              0.0, 0.0, 1.0, 0.0]
    return info
