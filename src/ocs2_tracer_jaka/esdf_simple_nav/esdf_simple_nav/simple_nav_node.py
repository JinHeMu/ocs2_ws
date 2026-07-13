#!/usr/bin/env python3

import heapq
import math
from typing import List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path

from tf2_ros import Buffer, TransformException, TransformListener


GridIndex = Tuple[int, int]  # (x, y)


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi)."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(q) -> float:
    """Quaternion -> planar yaw."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def set_pose_yaw(pose, yaw: float) -> None:
    """Set a geometry_msgs/Pose orientation from planar yaw."""
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = math.sin(0.5 * yaw)
    pose.orientation.w = math.cos(0.5 * yaw)


class EsdfSimpleNavigator(Node):
    """
    Minimal ROS 2 navigation node:

      /esdf_occ2d (OccupancyGrid)
                  |
                  v
               A* planner <--- /goal_pose (PoseStamped from RViz)
                  |
                  v
          /planned_path (Path)
                  |
                  v
           Pure Pursuit controller
                  |
                  v
              /cmd_vel (Twist)

    Robot pose is obtained from TF: map_frame -> base_frame.
    """

    def __init__(self):
        super().__init__("esdf_simple_navigator")

        # Topics / frames
        self.declare_parameter("map_topic", "/esdf_occ2d")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        # Grid / planning
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("unknown_is_occupied", True)
        self.declare_parameter("inflation_radius", 0.25)
        self.declare_parameter("allow_diagonal", True)
        self.declare_parameter("heuristic_weight", 1.0)
        self.declare_parameter("start_snap_radius", 0.30)

        # Controller
        self.declare_parameter("control_period", 0.05)
        self.declare_parameter("lookahead_distance", 0.40)
        self.declare_parameter("max_linear_speed", 0.30)
        self.declare_parameter("min_linear_speed", 0.05)
        self.declare_parameter("max_angular_speed", 1.00)
        self.declare_parameter("rotate_in_place_angle", 1.00)
        self.declare_parameter("goal_position_tolerance", 0.08)
        self.declare_parameter("goal_yaw_tolerance", 0.12)
        self.declare_parameter("replan_deviation", 0.60)

        self.map_topic = self._str_param("map_topic")
        self.goal_topic = self._str_param("goal_topic")
        self.path_topic = self._str_param("path_topic")
        self.cmd_vel_topic = self._str_param("cmd_vel_topic")
        self.map_frame = self._str_param("map_frame")
        self.base_frame = self._str_param("base_frame")

        self.occupied_threshold = self._int_param("occupied_threshold")
        self.unknown_is_occupied = self._bool_param("unknown_is_occupied")
        self.inflation_radius = self._float_param("inflation_radius")
        self.allow_diagonal = self._bool_param("allow_diagonal")
        self.heuristic_weight = self._float_param("heuristic_weight")
        self.start_snap_radius = self._float_param("start_snap_radius")

        self.control_period = self._float_param("control_period")
        self.lookahead_distance = self._float_param("lookahead_distance")
        self.max_linear_speed = self._float_param("max_linear_speed")
        self.min_linear_speed = self._float_param("min_linear_speed")
        self.max_angular_speed = self._float_param("max_angular_speed")
        self.rotate_in_place_angle = self._float_param("rotate_in_place_angle")
        self.goal_position_tolerance = self._float_param(
            "goal_position_tolerance"
        )
        self.goal_yaw_tolerance = self._float_param("goal_yaw_tolerance")
        self.replan_deviation = self._float_param("replan_deviation")

        # Map state: arrays are [y, x].
        self.map_ready = False
        self.grid: Optional[np.ndarray] = None
        self.obstacle_mask: Optional[np.ndarray] = None
        self.width = 0
        self.height = 0
        self.resolution = 0.0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_yaw = 0.0

        # Navigation state.
        self.goal_pose_map: Optional[Tuple[float, float, float]] = None
        self.path_xy: Optional[np.ndarray] = None
        self.progress_index = 0
        self.navigation_active = False
        self.need_replan = False
        self._tf_warned = False

        # TF
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Map QoS matches the TRANSIENT_LOCAL publisher in the user's map node.
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        path_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            map_qos,
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.goal_topic,
            self.goal_callback,
            10,
        )

        self.path_pub = self.create_publisher(
            Path,
            self.path_topic,
            path_qos,
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10,
        )

        self.control_timer = self.create_timer(
            self.control_period,
            self.control_loop,
        )

        self.get_logger().info("ESDF simple navigator started")
        self.get_logger().info(f"Map topic: {self.map_topic}")
        self.get_logger().info(f"Goal topic: {self.goal_topic}")
        self.get_logger().info(
            f"TF pose source: {self.map_frame} -> {self.base_frame}"
        )
        self.get_logger().info(f"Velocity output: {self.cmd_vel_topic}")

    # ------------------------------------------------------------------
    # Parameter helpers
    # ------------------------------------------------------------------

    def _str_param(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _float_param(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def _int_param(self, name: str) -> int:
        return self.get_parameter(name).get_parameter_value().integer_value

    def _bool_param(self, name: str) -> bool:
        return self.get_parameter(name).get_parameter_value().bool_value

    # ------------------------------------------------------------------
    # Map handling
    # ------------------------------------------------------------------

    def map_callback(self, msg: OccupancyGrid) -> None:
        first_map = not self.map_ready

        self.width = int(msg.info.width)
        self.height = int(msg.info.height)
        self.resolution = float(msg.info.resolution)

        if self.width <= 0 or self.height <= 0 or self.resolution <= 0.0:
            self.get_logger().error("Invalid OccupancyGrid geometry")
            return

        expected = self.width * self.height
        if len(msg.data) != expected:
            self.get_logger().error(
                f"Map data size mismatch: got {len(msg.data)}, expected {expected}"
            )
            return

        # OccupancyGrid data is row-major, so [height, width] == [y, x].
        self.grid = np.asarray(msg.data, dtype=np.int16).reshape(
            (self.height, self.width)
        )

        self.origin_x = float(msg.info.origin.position.x)
        self.origin_y = float(msg.info.origin.position.y)
        self.origin_yaw = quaternion_to_yaw(msg.info.origin.orientation)

        if msg.header.frame_id:
            self.map_frame = msg.header.frame_id

        occupied = self.grid >= self.occupied_threshold
        if self.unknown_is_occupied:
            occupied = np.logical_or(occupied, self.grid < 0)

        inflation_cells = int(
            math.ceil(self.inflation_radius / self.resolution)
        )
        self.obstacle_mask = self.inflate_obstacles(
            occupied,
            inflation_cells,
        )

        self.map_ready = True

        if first_map:
            occupied_count = int(np.count_nonzero(occupied))
            inflated_count = int(np.count_nonzero(self.obstacle_mask))
            self.get_logger().info(
                f"Map received: {self.width} x {self.height}, "
                f"resolution={self.resolution:.3f} m, "
                f"occupied={occupied_count}, inflated={inflated_count}"
            )

            if self.goal_pose_map is not None:
                self.need_replan = True

    @staticmethod
    def inflate_obstacles(mask: np.ndarray, radius_cells: int) -> np.ndarray:
        """Disk inflation using NumPy slicing, no SciPy dependency."""
        if radius_cells <= 0:
            return mask.copy()

        h, w = mask.shape
        inflated = mask.copy()

        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy > radius_cells * radius_cells:
                    continue

                src_y0 = max(0, -dy)
                src_y1 = min(h, h - dy)
                src_x0 = max(0, -dx)
                src_x1 = min(w, w - dx)

                dst_y0 = max(0, dy)
                dst_y1 = min(h, h + dy)
                dst_x0 = max(0, dx)
                dst_x1 = min(w, w + dx)

                if src_y0 >= src_y1 or src_x0 >= src_x1:
                    continue

                inflated[dst_y0:dst_y1, dst_x0:dst_x1] |= (
                    mask[src_y0:src_y1, src_x0:src_x1]
                )

        return inflated

    # ------------------------------------------------------------------
    # Coordinate transforms
    # ------------------------------------------------------------------

    def world_to_grid(self, x: float, y: float) -> GridIndex:
        """
        World/map coordinates -> grid index (ix, iy).

        Handles a non-zero yaw in OccupancyGrid.info.origin as well.
        """
        dx = x - self.origin_x
        dy = y - self.origin_y

        c = math.cos(self.origin_yaw)
        s = math.sin(self.origin_yaw)

        # Inverse rotation R^T.
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy

        ix = int(math.floor(local_x / self.resolution))
        iy = int(math.floor(local_y / self.resolution))
        return ix, iy

    def grid_to_world(self, ix: int, iy: int) -> Tuple[float, float]:
        """Grid cell center -> world/map coordinates."""
        local_x = (ix + 0.5) * self.resolution
        local_y = (iy + 0.5) * self.resolution

        c = math.cos(self.origin_yaw)
        s = math.sin(self.origin_yaw)

        x = self.origin_x + c * local_x - s * local_y
        y = self.origin_y + s * local_x + c * local_y
        return x, y

    def in_bounds(self, cell: GridIndex) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def get_robot_pose(self) -> Optional[Tuple[float, float, float]]:
        """Return (x, y, yaw) of base_frame in map_frame."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
            )
        except TransformException as exc:
            if not self._tf_warned:
                self.get_logger().warn(
                    f"Cannot get TF {self.map_frame} <- {self.base_frame}: {exc}"
                )
                self._tf_warned = True
            return None

        self._tf_warned = False
        x = float(tf.transform.translation.x)
        y = float(tf.transform.translation.y)
        yaw = quaternion_to_yaw(tf.transform.rotation)
        return x, y, yaw

    def goal_to_map(
        self,
        msg: PoseStamped,
    ) -> Optional[Tuple[float, float, float]]:
        """
        Transform a planar PoseStamped into map_frame.

        For ground robots, only x/y/yaw are used.
        """
        source_frame = msg.header.frame_id or self.map_frame

        gx = float(msg.pose.position.x)
        gy = float(msg.pose.position.y)
        gyaw = quaternion_to_yaw(msg.pose.orientation)

        if source_frame == self.map_frame:
            return gx, gy, gyaw

        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                source_frame,
                Time(),
            )
        except TransformException as exc:
            self.get_logger().error(
                f"Cannot transform goal {source_frame} -> {self.map_frame}: {exc}"
            )
            return None

        tx = float(tf.transform.translation.x)
        ty = float(tf.transform.translation.y)
        tyaw = quaternion_to_yaw(tf.transform.rotation)

        c = math.cos(tyaw)
        s = math.sin(tyaw)

        mx = tx + c * gx - s * gy
        my = ty + s * gx + c * gy
        myaw = normalize_angle(tyaw + gyaw)
        return mx, my, myaw

    # ------------------------------------------------------------------
    # Goal and planning
    # ------------------------------------------------------------------

    def goal_callback(self, msg: PoseStamped) -> None:
        goal = self.goal_to_map(msg)
        if goal is None:
            return

        self.stop_robot()
        self.navigation_active = False
        self.clear_path()

        self.goal_pose_map = goal
        self.need_replan = True

        self.get_logger().info(
            f"New goal: x={goal[0]:.3f}, y={goal[1]:.3f}, "
            f"yaw={goal[2]:.3f} rad"
        )

        # Try immediately. If TF or map is not ready, control_loop will retry.
        self.try_plan()

    def try_plan(self) -> bool:
        if not self.need_replan:
            return False

        if not self.map_ready or self.obstacle_mask is None:
            return False

        robot = self.get_robot_pose()
        if robot is None:
            return False

        if self.goal_pose_map is None:
            self.need_replan = False
            return False

        start_cell = self.world_to_grid(robot[0], robot[1])
        goal_cell = self.world_to_grid(
            self.goal_pose_map[0],
            self.goal_pose_map[1],
        )

        if not self.in_bounds(start_cell):
            self.get_logger().error(
                f"Robot is outside map: grid={start_cell}"
            )
            self.need_replan = False
            return False

        if not self.in_bounds(goal_cell):
            self.get_logger().error(
                f"Goal is outside map: grid={goal_cell}"
            )
            self.need_replan = False
            return False

        # The robot may lie inside an inflated region at startup. Snap only
        # the start to a nearby free cell; do not silently move the user's goal.
        if self.obstacle_mask[start_cell[1], start_cell[0]]:
            snapped = self.find_nearest_free(
                start_cell,
                self.start_snap_radius,
            )
            if snapped is None:
                self.get_logger().error(
                    "Start pose is occupied and no nearby free cell was found"
                )
                self.need_replan = False
                return False
            self.get_logger().warn(
                f"Start cell {start_cell} is inflated/occupied; "
                f"planning from nearby free cell {snapped}"
            )
            start_cell = snapped

        if self.obstacle_mask[goal_cell[1], goal_cell[0]]:
            self.get_logger().error(
                f"Goal cell {goal_cell} is occupied or inside inflation radius"
            )
            self.need_replan = False
            return False

        self.get_logger().info(
            f"Planning A*: start={start_cell}, goal={goal_cell}"
        )

        cell_path = self.astar(start_cell, goal_cell)
        if cell_path is None:
            self.get_logger().error("A* failed: no path found")
            self.need_replan = False
            self.stop_robot()
            return False

        points = [self.grid_to_world(x, y) for x, y in cell_path]

        # Connect the displayed/control path to the actual robot pose and exact goal.
        if points:
            points[0] = (robot[0], robot[1])

        exact_goal = (
            self.goal_pose_map[0],
            self.goal_pose_map[1],
        )
        if (
            not points
            or math.hypot(
                points[-1][0] - exact_goal[0],
                points[-1][1] - exact_goal[1],
            )
            > 1e-6
        ):
            points.append(exact_goal)

        self.path_xy = np.asarray(points, dtype=np.float64)
        self.progress_index = 0
        self.navigation_active = True
        self.need_replan = False

        self.publish_path()
        self.get_logger().info(
            f"Path ready: {len(cell_path)} grid cells, "
            f"{len(points)} control points"
        )
        return True

    def find_nearest_free(
        self,
        center: GridIndex,
        radius_m: float,
    ) -> Optional[GridIndex]:
        if self.obstacle_mask is None:
            return None

        cx, cy = center
        radius_cells = max(
            1,
            int(math.ceil(radius_m / self.resolution)),
        )

        best = None
        best_d2 = float("inf")

        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                d2 = dx * dx + dy * dy
                if d2 > radius_cells * radius_cells:
                    continue

                x = cx + dx
                y = cy + dy
                if not self.in_bounds((x, y)):
                    continue
                if self.obstacle_mask[y, x]:
                    continue

                if d2 < best_d2:
                    best_d2 = d2
                    best = (x, y)

        return best

    def astar(
        self,
        start: GridIndex,
        goal: GridIndex,
    ) -> Optional[List[GridIndex]]:
        if self.obstacle_mask is None:
            return None

        if start == goal:
            return [start]

        if self.allow_diagonal:
            neighbors = [
                (-1, 0, 1.0),
                (1, 0, 1.0),
                (0, -1, 1.0),
                (0, 1, 1.0),
                (-1, -1, math.sqrt(2.0)),
                (-1, 1, math.sqrt(2.0)),
                (1, -1, math.sqrt(2.0)),
                (1, 1, math.sqrt(2.0)),
            ]
        else:
            neighbors = [
                (-1, 0, 1.0),
                (1, 0, 1.0),
                (0, -1, 1.0),
                (0, 1, 1.0),
            ]

        g_score = np.full(
            (self.height, self.width),
            np.inf,
            dtype=np.float64,
        )
        closed = np.zeros(
            (self.height, self.width),
            dtype=np.bool_,
        )

        came_from = {}

        sx, sy = start
        gx, gy = goal

        g_score[sy, sx] = 0.0

        def heuristic(x: int, y: int) -> float:
            return math.hypot(gx - x, gy - y)

        open_heap = []
        heapq.heappush(
            open_heap,
            (
                self.heuristic_weight * heuristic(sx, sy),
                0.0,
                sx,
                sy,
            ),
        )

        while open_heap:
            _, current_g, x, y = heapq.heappop(open_heap)

            if closed[y, x]:
                continue

            if current_g > g_score[y, x] + 1e-12:
                continue

            closed[y, x] = True

            if (x, y) == goal:
                return self.reconstruct_path(
                    came_from,
                    start,
                    goal,
                )

            for dx, dy, step_cost in neighbors:
                nx = x + dx
                ny = y + dy

                if not self.in_bounds((nx, ny)):
                    continue
                if closed[ny, nx]:
                    continue
                if self.obstacle_mask[ny, nx]:
                    continue

                # Prevent diagonal corner cutting.
                if dx != 0 and dy != 0:
                    if (
                        self.obstacle_mask[y, nx]
                        or self.obstacle_mask[ny, x]
                    ):
                        continue

                tentative_g = current_g + step_cost

                if tentative_g + 1e-12 < g_score[ny, nx]:
                    g_score[ny, nx] = tentative_g
                    came_from[(nx, ny)] = (x, y)

                    f = (
                        tentative_g
                        + self.heuristic_weight * heuristic(nx, ny)
                    )
                    heapq.heappush(
                        open_heap,
                        (f, tentative_g, nx, ny),
                    )

        return None

    @staticmethod
    def reconstruct_path(
        came_from,
        start: GridIndex,
        goal: GridIndex,
    ) -> List[GridIndex]:
        path = [goal]
        current = goal

        while current != start:
            current = came_from[current]
            path.append(current)

        path.reverse()
        return path

    # ------------------------------------------------------------------
    # Path publication
    # ------------------------------------------------------------------

    def clear_path(self) -> None:
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        self.path_pub.publish(msg)
        self.path_xy = None
        self.progress_index = 0

    def publish_path(self) -> None:
        if self.path_xy is None or self.goal_pose_map is None:
            return

        msg = Path()
        stamp = self.get_clock().now().to_msg()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame

        n = len(self.path_xy)

        for i in range(n):
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = self.map_frame
            pose.pose.position.x = float(self.path_xy[i, 0])
            pose.pose.position.y = float(self.path_xy[i, 1])
            pose.pose.position.z = 0.0

            if i + 1 < n:
                dx = self.path_xy[i + 1, 0] - self.path_xy[i, 0]
                dy = self.path_xy[i + 1, 1] - self.path_xy[i, 1]
                yaw = math.atan2(dy, dx)
            else:
                yaw = self.goal_pose_map[2]

            set_pose_yaw(pose.pose, yaw)
            msg.poses.append(pose)

        self.path_pub.publish(msg)

    # ------------------------------------------------------------------
    # Pure Pursuit controller
    # ------------------------------------------------------------------

    def control_loop(self) -> None:
        if self.need_replan:
            self.try_plan()

        if (
            not self.navigation_active
            or self.path_xy is None
            or self.goal_pose_map is None
        ):
            return

        robot = self.get_robot_pose()
        if robot is None:
            self.stop_robot()
            return

        rx, ry, ryaw = robot
        goal_x, goal_y, goal_yaw = self.goal_pose_map

        distance_to_goal = math.hypot(
            goal_x - rx,
            goal_y - ry,
        )

        # Final position reached: rotate to requested final yaw.
        if distance_to_goal <= self.goal_position_tolerance:
            yaw_error = normalize_angle(goal_yaw - ryaw)

            if abs(yaw_error) <= self.goal_yaw_tolerance:
                self.stop_robot()
                self.navigation_active = False
                self.get_logger().info("Goal reached")
                return

            cmd = Twist()
            cmd.angular.z = float(
                np.clip(
                    1.5 * yaw_error,
                    -self.max_angular_speed,
                    self.max_angular_speed,
                )
            )
            self.cmd_pub.publish(cmd)
            return

        # Find nearest path point, but do not allow large backward jumps.
        search_start = max(0, self.progress_index - 5)
        remaining = self.path_xy[search_start:]

        delta = remaining - np.array([rx, ry])
        d2 = np.einsum("ij,ij->i", delta, delta)
        nearest = search_start + int(np.argmin(d2))
        nearest_distance = math.sqrt(float(np.min(d2)))

        if nearest > self.progress_index:
            self.progress_index = nearest

        # Large path deviation -> stop and replan from current TF pose.
        if nearest_distance > self.replan_deviation:
            self.get_logger().warn(
                f"Path deviation {nearest_distance:.2f} m; replanning"
            )
            self.stop_robot()
            self.navigation_active = False
            self.need_replan = True
            return

        target_index = self.select_lookahead_index(
            rx,
            ry,
            self.progress_index,
        )
        target_x = float(self.path_xy[target_index, 0])
        target_y = float(self.path_xy[target_index, 1])

        dx = target_x - rx
        dy = target_y - ry

        c = math.cos(ryaw)
        s = math.sin(ryaw)

        # Target coordinates in robot base frame.
        x_r = c * dx + s * dy
        y_r = -s * dx + c * dy

        heading_error = math.atan2(y_r, x_r)

        cmd = Twist()

        # If target is too far to the side/behind, rotate before driving.
        if (
            abs(heading_error) > self.rotate_in_place_angle
            or x_r < 0.0
        ):
            cmd.linear.x = 0.0
            cmd.angular.z = float(
                np.clip(
                    1.5 * heading_error,
                    -self.max_angular_speed,
                    self.max_angular_speed,
                )
            )
            self.cmd_pub.publish(cmd)
            return

        # Slow down for large heading error and near the goal.
        heading_scale = max(
            0.25,
            1.0 - abs(heading_error) / self.rotate_in_place_angle,
        )

        distance_speed = max(
            self.min_linear_speed,
            min(self.max_linear_speed, 0.8 * distance_to_goal),
        )

        linear_speed = min(
            self.max_linear_speed,
            distance_speed * heading_scale,
        )

        lookahead_sq = max(
            dx * dx + dy * dy,
            1e-6,
        )
        curvature = 2.0 * y_r / lookahead_sq
        angular_speed = linear_speed * curvature

        cmd.linear.x = float(linear_speed)
        cmd.angular.z = float(
            np.clip(
                angular_speed,
                -self.max_angular_speed,
                self.max_angular_speed,
            )
        )
        self.cmd_pub.publish(cmd)

    def select_lookahead_index(
        self,
        rx: float,
        ry: float,
        start_index: int,
    ) -> int:
        assert self.path_xy is not None

        n = len(self.path_xy)
        if n == 1:
            return 0

        idx = min(max(start_index, 0), n - 1)

        accumulated = math.hypot(
            self.path_xy[idx, 0] - rx,
            self.path_xy[idx, 1] - ry,
        )

        while idx + 1 < n and accumulated < self.lookahead_distance:
            segment = math.hypot(
                self.path_xy[idx + 1, 0] - self.path_xy[idx, 0],
                self.path_xy[idx + 1, 1] - self.path_xy[idx, 1],
            )
            accumulated += segment
            idx += 1

        return idx

    def stop_robot(self) -> None:
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = EsdfSimpleNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
