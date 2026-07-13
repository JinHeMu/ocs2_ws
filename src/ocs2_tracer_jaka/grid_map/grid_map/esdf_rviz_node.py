#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    HistoryPolicy,
    ReliabilityPolicy,
    DurabilityPolicy,
)

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose


class EsdfRvizPublisher(Node):
    def __init__(self):
        super().__init__("esdf_rviz_publisher")

        self.declare_parameter("esdf_file", "maps/tracer_jaka_zu5_scene_esdf.npz")
        self.declare_parameter("frame_id", "map")

        # z_slice < 0 表示显示完整 3D ESDF
        # z_slice >= 0 表示只显示某个高度的切片
        self.declare_parameter("z_slice", 0.1)

        self.declare_parameter("max_distance", 0.8)
        self.declare_parameter("stride", 2)
        self.declare_parameter("publish_period", 0.5)

        # 2D 投影高度范围，适合移动底盘
        self.declare_parameter("z_min_2d", 0.05)
        self.declare_parameter("z_max_2d", 0.6)

        self.esdf_file = (
            self.get_parameter("esdf_file")
            .get_parameter_value()
            .string_value
        )
        self.frame_id = (
            self.get_parameter("frame_id")
            .get_parameter_value()
            .string_value
        )
        self.z_slice = (
            self.get_parameter("z_slice")
            .get_parameter_value()
            .double_value
        )
        self.max_distance = (
            self.get_parameter("max_distance")
            .get_parameter_value()
            .double_value
        )
        self.stride = (
            self.get_parameter("stride")
            .get_parameter_value()
            .integer_value
        )
        self.publish_period = (
            self.get_parameter("publish_period")
            .get_parameter_value()
            .double_value
        )
        self.z_min_2d = (
            self.get_parameter("z_min_2d")
            .get_parameter_value()
            .double_value
        )
        self.z_max_2d = (
            self.get_parameter("z_max_2d")
            .get_parameter_value()
            .double_value
        )

        if self.stride < 1:
            self.stride = 1

        data = np.load(self.esdf_file)

        self.esdf = data["esdf"]
        self.occ = data["occupancy"]
        self.origin = data["origin"].astype(float)
        self.voxel_size = float(data["voxel_size"])

        # ROS2 中 latch 的近似等价是 transient_local durability。
        # 这样 RViz2 后打开时也能收到最后一次发布的地图。
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.cloud_pub = self.create_publisher(
            PointCloud2,
            "/esdf_cloud",
            qos,
        )

        self.grid_pub = self.create_publisher(
            OccupancyGrid,
            "/esdf_occ2d",
            qos,
        )

        self.timer = self.create_timer(
            self.publish_period,
            self.publish_maps,
        )

        self.get_logger().info(f"Loaded ESDF file: {self.esdf_file}")
        self.get_logger().info(f"ESDF shape: {self.esdf.shape}")
        self.get_logger().info(f"Origin: {self.origin}")
        self.get_logger().info(f"Voxel size: {self.voxel_size}")
        self.get_logger().info(f"Frame id: {self.frame_id}")

    def make_header(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        return header

    def make_esdf_cloud(self):
        """
        发布 PointCloud2:
          x, y, z: 体素中心坐标
          intensity: ESDF 值，单位 m
        """
        esdf = self.esdf
        origin = self.origin
        voxel_size = self.voxel_size
        stride = self.stride
        max_distance = self.max_distance

        nx, ny, nz = esdf.shape

        if self.z_slice >= 0.0:
            k = int(round((self.z_slice - origin[2]) / voxel_size))
            k = int(np.clip(k, 0, nz - 1))

            esdf_slice = esdf[::stride, ::stride, k]

            ii, jj = np.meshgrid(
                np.arange(0, nx, stride),
                np.arange(0, ny, stride),
                indexing="ij",
            )

            dd = esdf_slice

            mask = np.abs(dd) <= max_distance

            xs = origin[0] + (ii[mask] + 0.5) * voxel_size
            ys = origin[1] + (jj[mask] + 0.5) * voxel_size
            zs = np.full_like(xs, origin[2] + (k + 0.5) * voxel_size)
            intensities = dd[mask].astype(np.float32)

            points = np.column_stack((xs, ys, zs, intensities)).astype(np.float32)

        else:
            esdf_sampled = esdf[::stride, ::stride, ::stride]

            ii, jj, kk = np.meshgrid(
                np.arange(0, nx, stride),
                np.arange(0, ny, stride),
                np.arange(0, nz, stride),
                indexing="ij",
            )

            dd = esdf_sampled
            mask = np.abs(dd) <= max_distance

            xs = origin[0] + (ii[mask] + 0.5) * voxel_size
            ys = origin[1] + (jj[mask] + 0.5) * voxel_size
            zs = origin[2] + (kk[mask] + 0.5) * voxel_size
            intensities = dd[mask].astype(np.float32)

            points = np.column_stack((xs, ys, zs, intensities)).astype(np.float32)

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        return point_cloud2.create_cloud(
            self.make_header(),
            fields,
            points.tolist(),
        )

    def make_2d_occupancy_grid(self):
        """
        把 3D occupancy 在 z_min_2d ~ z_max_2d 内投影成 2D OccupancyGrid。
        对移动底盘导航/可视化比较友好。
        """
        occ = self.occ
        origin = self.origin
        voxel_size = self.voxel_size

        nx, ny, nz = occ.shape

        k0 = int(np.floor((self.z_min_2d - origin[2]) / voxel_size))
        k1 = int(np.ceil((self.z_max_2d - origin[2]) / voxel_size))

        k0 = max(k0, 0)
        k1 = min(k1, nz)

        occ2d = occ[:, :, k0:k1].any(axis=2)

        grid = OccupancyGrid()
        grid.header = self.make_header()
        grid.info.map_load_time = self.get_clock().now().to_msg()
        grid.info.resolution = float(voxel_size)
        grid.info.width = int(nx)
        grid.info.height = int(ny)

        grid.info.origin = Pose()
        grid.info.origin.position.x = float(origin[0])
        grid.info.origin.position.y = float(origin[1])
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0

        # OccupancyGrid 是 row-major:
        # index = x + y * width
        # occ2d shape 是 [x, y]，所以转置成 [y, x] 再 flatten。
        data_2d = np.where(occ2d.T, 100, 0).astype(np.int8)

        grid.data = data_2d.flatten().tolist()
        return grid

    def publish_maps(self):
        cloud_msg = self.make_esdf_cloud()
        grid_msg = self.make_2d_occupancy_grid()

        self.cloud_pub.publish(cloud_msg)
        self.grid_pub.publish(grid_msg)


def main(args=None):
    rclpy.init(args=args)

    node = EsdfRvizPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
