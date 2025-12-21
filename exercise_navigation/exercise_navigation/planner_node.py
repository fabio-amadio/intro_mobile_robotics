#!/usr/bin/env python3
"""
A simple grid-based global planner node for ROS 2 (Python / rclpy).

- Subscribes:  /map (nav_msgs/OccupancyGrid), /goal_pose (geometry_msgs/PoseStamped)
- Publishes:   /plan (nav_msgs/Path)
- Uses TF:     map -> base_link (or map -> <base_frame>) to get the start pose

Algorithm:
- Builds a binary occupancy grid from the map (free / occupied).
- Inflates obstacles by a configurable radius.
- Runs A* (optionally with diagonal moves).
- Converts the cell path into a world-frame Path message.

Usage:
  ros2 run exercise_navigation planner --ros-args -p inflation_radius:=0.20
  (Make sure something publishes /map and TF map->base_link, and send a goal on /goal_pose.)
"""

from __future__ import annotations

import math
import heapq
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time

import tf2_ros

GridIndex = Tuple[int, int]  # (mx, my)


@dataclass
class MapMeta:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    frame_id: str


def clamp(v: int, lo: int, hi: int) -> int:
    return lo if v < lo else hi if v > hi else v


def yaw_from_quat_xyzw(x, y, z, w):
    # yaw (Z rotation) from quaternion (x,y,z,w)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class AStarPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("simple_astar_planner")

        # -------------------------
        # Parameters (tweakable)
        # -------------------------
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("path_topic", "/plan")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        # Occupancy thresholds: Occupied if value >= occupied_threshold
        self.declare_parameter("occupied_threshold", 50)  # 0..100
        self.declare_parameter("treat_unknown_as_obstacle", True)

        # Obstacle inflation (meters)
        self.declare_parameter("inflation_radius", 0.20)

        # A* options
        self.declare_parameter("allow_diagonal", True)

        # How often to (re)plan when we have a goal (Hz). 0 disables timer replanning.
        self.declare_parameter("replan_rate_hz", 0.0)

        # TF lookup timeout (seconds)
        self.declare_parameter("tf_timeout_sec", 0.2)

        # Path publishing: downsample every N cells (1 = no downsample)
        self.declare_parameter("path_downsample", 1)

        # -------------------------
        # QoS for /map (latched in many setups)
        # -------------------------
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.get_parameter("map_topic").value,
            self._on_map,
            map_qos,
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.get_parameter("goal_topic").value,
            self._on_goal,
            10,
        )

        self.path_pub = self.create_publisher(
            Path,
            self.get_parameter("path_topic").value,
            10,
        )

        # TF2
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Internal state
        self._map_msg: Optional[OccupancyGrid] = None
        self._meta: Optional[MapMeta] = None
        self._occ_raw: Optional[List[int]] = None            # original occupancy values (flattened)
        self._grid_occ: Optional[List[int]] = None           # binary occupancy (0 free, 1 occ)
        self._grid_inflated: Optional[List[int]] = None      # inflated binary occupancy
        self._inflation_offsets: List[GridIndex] = []

        self._goal_msg: Optional[PoseStamped] = None
        self._have_goal: bool = False

        self._replan_timer = None
        replan_rate = float(self.get_parameter("replan_rate_hz").value)
        if replan_rate > 0.0:
            self._replan_timer = self.create_timer(1.0 / replan_rate, self._on_timer)

        self.get_logger().info("Planner node started. Waiting for /map and goals...")

    # -------------------------
    # Callbacks
    # -------------------------
    def _on_map(self, msg: OccupancyGrid) -> None:
        if msg.info.width == 0 or msg.info.height == 0 or not msg.data:
            self.get_logger().warn("Received empty map; ignoring.")
            return

        self._map_msg = msg
        self._meta = MapMeta(
            width=int(msg.info.width),
            height=int(msg.info.height),
            resolution=float(msg.info.resolution),
            origin_x=float(msg.info.origin.position.x),
            origin_y=float(msg.info.origin.position.y),
            frame_id=msg.header.frame_id if msg.header.frame_id else self.get_parameter("map_frame").value,
        )

        self._occ_raw = list(msg.data)
        self._grid_occ = self._build_binary_occupancy(self._occ_raw, self._meta)
        self._prepare_inflation(self._meta)

        self._grid_inflated = self._inflate_grid(self._grid_occ, self._meta)

        self.get_logger().info(
            f"Map received: {self._meta.width}x{self._meta.height}, res={self._meta.resolution:.3f} m/cell, "
            f"inflation_radius={float(self.get_parameter('inflation_radius').value):.2f} m"
        )

        # If we already have a goal, replan now that map is available/updated.
        if self._have_goal:
            self._plan_and_publish()

    def _on_goal(self, msg: PoseStamped) -> None:
        self._goal_msg = msg
        self._have_goal = True
        self.get_logger().info(
            f"Goal received in frame '{msg.header.frame_id or '(empty)'}': "
            f"x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}"
        )
        self._plan_and_publish()

    def _on_timer(self) -> None:
        # Periodic replanning (optional)
        if self._have_goal:
            self._plan_and_publish()

    # -------------------------
    # Map processing
    # -------------------------
    def _build_binary_occupancy(self, occ_values: List[int], meta: MapMeta) -> List[int]:
        """
        Convert OccupancyGrid data (0..100, -1 unknown) to binary occupancy:
          0 = free
          1 = occupied
        """
        occ_th = int(self.get_parameter("occupied_threshold").value)
        unknown_as_obs = bool(self.get_parameter("treat_unknown_as_obstacle").value)

        out = [0] * (meta.width * meta.height)
        for i, v in enumerate(occ_values):
            if v < 0:
                out[i] = 1 if unknown_as_obs else 0
            else:
                out[i] = 1 if v >= occ_th else 0
        return out

    def _prepare_inflation(self, meta: MapMeta) -> None:
        """
        Precompute integer (dx, dy) offsets for a circular inflation kernel.
        """
        radius_m = float(self.get_parameter("inflation_radius").value)
        radius_cells = int(math.ceil(radius_m / meta.resolution))

        offsets: List[GridIndex] = []
        r2 = radius_cells * radius_cells
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy <= r2:
                    offsets.append((dx, dy))
        self._inflation_offsets = offsets

    def _inflate_grid(self, grid_occ: List[int], meta: MapMeta) -> List[int]:
        """
        Inflate obstacles: any cell within inflation_radius of an occupied cell becomes occupied.
        """
        inflated = grid_occ.copy()
        w, h = meta.width, meta.height

        # Collect occupied cells (for speed and clarity)
        occ_cells: List[GridIndex] = []
        for idx, v in enumerate(grid_occ):
            if v == 1:
                x = idx % w
                y = idx // w
                occ_cells.append((x, y))

        for (ox, oy) in occ_cells:
            for (dx, dy) in self._inflation_offsets:
                nx = ox + dx
                ny = oy + dy
                if 0 <= nx < w and 0 <= ny < h:
                    inflated[ny * w + nx] = 1

        return inflated

    # -------------------------
    # World <-> Map index helpers
    # -------------------------
    def world_to_map(self, wx: float, wy: float, meta: MapMeta) -> Optional[GridIndex]:
        """
        Convert world (map frame) coordinates to integer map cell indices.
        Returns None if outside map bounds.
        """
        mx = int(math.floor((wx - meta.origin_x) / meta.resolution))
        my = int(math.floor((wy - meta.origin_y) / meta.resolution))
        if mx < 0 or my < 0 or mx >= meta.width or my >= meta.height:
            return None
        return (mx, my)

    def map_to_world(self, mx: int, my: int, meta: MapMeta) -> Tuple[float, float]:
        """
        Convert map cell indices to world coordinates (cell center).
        """
        wx = meta.origin_x + (mx + 0.5) * meta.resolution
        wy = meta.origin_y + (my + 0.5) * meta.resolution
        return wx, wy

    # -------------------------
    # TF start pose
    # -------------------------
    def _get_robot_pose_in_map(self) -> Optional[Tuple[float, float, float]]:
        """
        Returns (x, y, yaw) in map frame using TF lookup: map -> base_frame.
        """
        meta = self._meta
        if meta is None:
            return None

        map_frame = str(self.get_parameter("map_frame").value) or meta.frame_id
        base_frame = str(self.get_parameter("base_frame").value)
        timeout_sec = float(self.get_parameter("tf_timeout_sec").value)

        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame=map_frame,
                source_frame=base_frame,
                time=Time(),  # latest available
                timeout=Duration(seconds=timeout_sec),
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed ({map_frame} <- {base_frame}): {e}")
            return None

        x = float(tf.transform.translation.x)
        y = float(tf.transform.translation.y)
        q = tf.transform.rotation
        yaw = yaw_from_quat_xyzw(float(q.x), float(q.y), float(q.z), float(q.w))
        return (x, y, yaw)

    # -------------------------
    # Planning
    # -------------------------
    def _plan_and_publish(self) -> None:
        if self._meta is None or self._grid_inflated is None:
            self.get_logger().warn("No map yet; cannot plan.")
            return
        if self._goal_msg is None:
            self.get_logger().warn("No goal yet; cannot plan.")
            return

        start_pose = self._get_robot_pose_in_map()
        if start_pose is None:
            self.get_logger().warn("No robot pose in map frame; cannot plan.")
            return
        sx, sy, _ = start_pose

        meta = self._meta
        goal = self._goal_msg

        # Goal pose frame handling:
        # For teaching simplicity we assume goal is already in map frame.
        # If goal.header.frame_id is different, you'd transform it via TF here.
        goal_frame = goal.header.frame_id or meta.frame_id
        map_frame = str(self.get_parameter("map_frame").value) or meta.frame_id
        if goal_frame != map_frame:
            self.get_logger().warn(
                f"Goal is in frame '{goal_frame}' but planner expects '{map_frame}'. "
                "Publish goals in the map frame or add TF transforming here."
            )

        gmxmy = self.world_to_map(float(goal.pose.position.x), float(goal.pose.position.y), meta)
        smxmy = self.world_to_map(float(sx), float(sy), meta)

        if smxmy is None:
            self.get_logger().warn("Start is outside map bounds.")
            return
        if gmxmy is None:
            self.get_logger().warn("Goal is outside map bounds.")
            return

        start = smxmy
        goal_cell = gmxmy

        # If start or goal is in an inflated obstacle cell, stop early.
        if self._is_occupied(start, meta, inflated=True):
            self.get_logger().warn("Start cell is occupied (after inflation).")
            return
        if self._is_occupied(goal_cell, meta, inflated=True):
            self.get_logger().warn("Goal cell is occupied (after inflation).")
            return

        allow_diag = bool(self.get_parameter("allow_diagonal").value)
        path_cells = self._astar(start, goal_cell, meta, allow_diag)

        if not path_cells:
            self.get_logger().warn("A* failed to find a path.")
            return

        downsample = int(self.get_parameter("path_downsample").value)
        if downsample < 1:
            downsample = 1

        path_msg = self._cells_to_path(path_cells, meta, downsample=downsample)
        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Published path with {len(path_msg.poses)} poses.")

    def _is_occupied(self, cell: GridIndex, meta: MapMeta, inflated: bool = True) -> bool:
        mx, my = cell
        idx = my * meta.width + mx
        grid = self._grid_inflated if inflated else self._grid_occ
        if grid is None:
            return True
        return grid[idx] == 1

    def _neighbors(self, cell: GridIndex, meta: MapMeta, allow_diag: bool) -> List[Tuple[GridIndex, float]]:
        """
        Returns list of (neighbor_cell, step_cost).
        """
        x, y = cell
        nbrs: List[Tuple[GridIndex, float]] = []

        # 4-connected
        steps = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0)]
        # 8-connected
        if allow_diag:
            diag_cost = math.sqrt(2.0)
            steps += [(-1, -1, diag_cost), (-1, 1, diag_cost), (1, -1, diag_cost), (1, 1, diag_cost)]

        for dx, dy, c in steps:
            nx, ny = x + dx, y + dy
            if 0 <= nx < meta.width and 0 <= ny < meta.height:
                nbrs.append(((nx, ny), c))

        return nbrs

    def _heuristic(self, a: GridIndex, b: GridIndex, allow_diag: bool) -> float:
        """
        Heuristic: Euclidean (works well with diagonals); Manhattan also OK.
        """
        ax, ay = a
        bx, by = b
        dx = abs(ax - bx)
        dy = abs(ay - by)
        if allow_diag:
            # Octile distance (nice for 8-connected grids)
            f = (math.sqrt(2.0) - 1.0)
            return f * min(dx, dy) + max(dx, dy)
        return float(dx + dy)

    def _astar(self, start: GridIndex, goal: GridIndex, meta: MapMeta, allow_diag: bool) -> List[GridIndex]:
        """
        A* on inflated occupancy grid.
        Returns a list of cells from start to goal (inclusive), or [] if no path.
        """
        # Priority queue entries: (f_score, g_score, (x,y))
        open_heap: List[Tuple[float, float, GridIndex]] = []
        heapq.heappush(open_heap, (0.0, 0.0, start))

        came_from: Dict[GridIndex, GridIndex] = {}
        g_score: Dict[GridIndex, float] = {start: 0.0}

        closed: set[GridIndex] = set()

        while open_heap:
            f, g, current = heapq.heappop(open_heap)

            if current in closed:
                continue
            closed.add(current)

            if current == goal:
                return self._reconstruct_path(came_from, current)

            for (nbr, step_cost) in self._neighbors(current, meta, allow_diag):
                if nbr in closed:
                    continue
                if self._is_occupied(nbr, meta, inflated=True):
                    continue

                tentative_g = g_score[current] + step_cost

                # If better path found
                if nbr not in g_score or tentative_g < g_score[nbr]:
                    g_score[nbr] = tentative_g
                    came_from[nbr] = current
                    h = self._heuristic(nbr, goal, allow_diag)
                    f_new = tentative_g + h
                    heapq.heappush(open_heap, (f_new, tentative_g, nbr))

        return []

    def _reconstruct_path(self, came_from: Dict[GridIndex, GridIndex], current: GridIndex) -> List[GridIndex]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def _cells_to_path(self, cells: List[GridIndex], meta: MapMeta, downsample: int = 1) -> Path:
        """
        Convert a list of map cells to nav_msgs/Path in map frame.
        Orientation is left as identity (yaw not set) for simplicity.
        """
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("map_frame").value) or meta.frame_id

        # Downsample path cells (keep start + end)
        if downsample > 1 and len(cells) > 2:
            sampled = [cells[0]] + cells[1:-1:downsample] + [cells[-1]]
        else:
            sampled = cells

        for (mx, my) in sampled:
            wx, wy = self.map_to_world(mx, my, meta)
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(wx)
            ps.pose.position.y = float(wy)
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0  # no yaw for now
            msg.poses.append(ps)

        return msg


def main() -> None:
    rclpy.init()
    node = AStarPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
