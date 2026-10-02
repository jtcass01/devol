"""RRT and RRT* planners for a rectangular diff-drive robot on a 2D occupancy grid.

Ported from the Robot Motion Planning (535.622) final project MATLAB code
(RRT.m, RRTstar.m, gammaRRT.m, shortcutPath.m). The main differences:

* Collision checking is done in the workspace: the robot's rectangular
  footprint is placed at a pose and tested against the raw (un-inflated)
  occupancy grid, e.g. octomap_server's projected_map. No C-space map is built.
* Edges are "turn in place, then drive straight". The robot always faces along
  the segment it is driving, and a skid-steer base can rotate in place, so
  every edge in the tree is something the base can actually execute. Each node
  stores its arrival heading, and the rotation sweep at a node is collision
  checked whenever a child edge leaves it.
* Cost is path length. Turning is a feasibility constraint, not a cost, so the
  RRT* rewiring keeps its optimality argument.

The module is ROS-free so it can be unit tested offline.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from math import atan2, ceil, hypot, log, pi, sqrt
from typing import List, Optional, Tuple

import numpy as np
from scipy.ndimage import distance_transform_edt

__author__ = "Jacob Taylor Cassady"
__email__ = "jcassad1@jh.edu"

Pose = Tuple[float, float, float]


def wrap_to_pi(angle: float) -> float:
    return (angle + pi) % (2.0 * pi) - pi


class FootprintCollisionChecker:
    """Checks a rectangular robot footprint against an occupancy grid.

    The grid follows nav_msgs/OccupancyGrid conventions: row-major,
    grid[row, col] with row along +y and col along +x from the origin,
    values 0 free, 100 occupied, -1 unknown.
    """

    def __init__(self,
                 grid: np.ndarray,
                 resolution: float,
                 origin: Tuple[float, float],
                 length: float = 0.99,
                 width: float = 0.67,
                 padding: float = 0.05,
                 center_offset: float = 0.0,
                 occupied_threshold: int = 50,
                 unknown_is_occupied: bool = True):
        self.resolution = float(resolution)
        self.origin_x = float(origin[0])
        self.origin_y = float(origin[1])
        self.n_rows, self.n_cols = grid.shape

        occupied = grid >= occupied_threshold
        if unknown_is_occupied:
            occupied |= grid < 0
        self.occupied = occupied

        # Distance (m) from each cell centre to the nearest occupied cell centre,
        # with everything outside the map counted as occupied.
        padded = np.pad(~occupied, 1, constant_values=False)
        self._clearance = distance_transform_edt(padded)[1:-1, 1:-1] * self.resolution

        self.half_length = 0.5 * length + padding
        self.half_width = 0.5 * width + padding
        self.center_offset = center_offset  # footprint centre ahead of base_link (m)

        # Circle radii about the footprint centre. A cell is a square, so its
        # centre may sit up to half a diagonal from an occupied area.
        half_cell_diag = 0.5 * sqrt(2.0) * self.resolution
        self.inscribed_radius = min(self.half_length, self.half_width)
        self.circumscribed_radius = hypot(self.half_length, self.half_width)
        self._surely_free = self.circumscribed_radius + 2.0 * half_cell_diag
        self._surely_hit = self.inscribed_radius - 2.0 * half_cell_diag

        # Body-frame sample points covering the footprint at half-cell spacing
        # (boundary included), so every cell the rectangle overlaps is sampled.
        step = 0.5 * self.resolution
        nx = int(ceil(2.0 * self.half_length / step)) + 1
        ny = int(ceil(2.0 * self.half_width / step)) + 1
        xs = np.linspace(-self.half_length, self.half_length, nx) + center_offset
        ys = np.linspace(-self.half_width, self.half_width, ny)
        bx, by = np.meshgrid(xs, ys)
        self._body_points = np.stack([bx.ravel(), by.ravel()])  # 2 x M

        # Step sizes that keep every footprint point moving <= half a cell.
        self.translation_step = 0.5 * self.resolution
        reach = hypot(self.half_length + abs(center_offset), self.half_width)
        self.rotation_step = 0.5 * self.resolution / reach

    # Grid helpers
    @property
    def x_bounds(self) -> Tuple[float, float]:
        return self.origin_x, self.origin_x + self.n_cols * self.resolution

    @property
    def y_bounds(self) -> Tuple[float, float]:
        return self.origin_y, self.origin_y + self.n_rows * self.resolution

    def world_to_cells(self, xs: np.ndarray, ys: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        cols = np.floor((xs - self.origin_x) / self.resolution).astype(np.int64)
        rows = np.floor((ys - self.origin_y) / self.resolution).astype(np.int64)
        inside = (rows >= 0) & (rows < self.n_rows) & (cols >= 0) & (cols < self.n_cols)
        return rows, cols, inside

    def clearance(self, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        rows, cols, inside = self.world_to_cells(np.asarray(xs, float), np.asarray(ys, float))
        out = np.zeros(rows.shape)
        out[inside] = self._clearance[rows[inside], cols[inside]]
        return out

    # Collision checks
    def is_collision(self, x: float, y: float, theta: float) -> bool:
        """True if the footprint at (x, y, theta) overlaps an occupied cell or leaves the map."""
        return self.poses_in_collision(np.array([[x, y, theta]]))

    def poses_in_collision(self, poses: np.ndarray) -> bool:
        """True if any pose in an N x 3 array is in collision."""
        poses = np.atleast_2d(np.asarray(poses, dtype=float))
        if poses.size == 0:
            return False
        cx = poses[:, 0] + self.center_offset * np.cos(poses[:, 2])
        cy = poses[:, 1] + self.center_offset * np.sin(poses[:, 2])
        clear = self.clearance(cx, cy)

        # Cheap circle tests on the distance transform first.
        if np.any(clear < self._surely_hit):
            return True
        undecided = clear < self._surely_free
        if not undecided.any():
            return False

        sub = poses[undecided]
        c, s = np.cos(sub[:, 2]), np.sin(sub[:, 2])
        bx, by = self._body_points
        wx = sub[:, 0:1] + c[:, None] * bx[None, :] - s[:, None] * by[None, :]
        wy = sub[:, 1:2] + s[:, None] * bx[None, :] + c[:, None] * by[None, :]
        rows, cols, inside = self.world_to_cells(wx, wy)
        if not inside.all():
            return True
        return bool(self.occupied[rows, cols].any())

    def segment_poses(self, x0: float, y0: float, x1: float, y1: float, heading: float) -> np.ndarray:
        """Poses along a straight drive from (x0, y0) to (x1, y1) facing `heading`."""
        dist = hypot(x1 - x0, y1 - y0)
        n = max(1, int(ceil(dist / self.translation_step)))
        alpha = np.linspace(0.0, 1.0, n + 1)
        return np.stack([x0 + alpha * (x1 - x0),
                         y0 + alpha * (y1 - y0),
                         np.full(n + 1, heading)], axis=1)

    def rotation_poses(self, x: float, y: float, theta0: float, theta1: float) -> np.ndarray:
        """Poses for an in-place rotation from theta0 to theta1 (shortest direction)."""
        dtheta = wrap_to_pi(theta1 - theta0)
        n = max(1, int(ceil(abs(dtheta) / self.rotation_step)))
        alpha = np.linspace(0.0, 1.0, n + 1)
        return np.stack([np.full(n + 1, x), np.full(n + 1, y), theta0 + alpha * dtheta], axis=1)

    def segment_in_collision(self, x0, y0, x1, y1, heading) -> bool:
        return self.poses_in_collision(self.segment_poses(x0, y0, x1, y1, heading))

    def rotation_in_collision(self, x, y, theta0, theta1) -> bool:
        if abs(wrap_to_pi(theta1 - theta0)) < 1e-9:
            return False
        return self.poses_in_collision(self.rotation_poses(x, y, theta0, theta1))

    def free_area(self) -> float:
        return float((~self.occupied).sum()) * self.resolution ** 2


@dataclass
class PlannerConfig:
    algorithm: str = 'rrt_star'      # 'rrt' or 'rrt_star'
    step_size: float = 1.0           # max edge length (m)
    goal_bias: float = 0.1           # probability of sampling the goal
    goal_tolerance: float = 1.0      # try to connect to the goal from nodes this close (m)
    max_iterations: int = 20000
    max_planning_time: float = 2.0   # s; RRT* returns its best path when this runs out
    patience: int = 1500             # RRT*: stop after this many iterations without improvement
    gamma_scale: float = 1.2         # RRT*: multiple of the theoretical gamma*
    max_rewire_radius: float = 3.0   # m
    shortcut_attempts: int = 200
    require_goal_yaw: bool = True    # rotate to the goal yaw at the end of the path
    seed: Optional[int] = None


@dataclass
class PlanResult:
    path: List[Pose] = field(default_factory=list)   # (x, y, heading) waypoints, start first
    raw_path: List[Pose] = field(default_factory=list)
    cost: float = float('inf')
    iterations: int = 0
    nodes: int = 0
    planning_time: float = 0.0
    first_solution_time: Optional[float] = None
    tree: Optional[Tuple[np.ndarray, np.ndarray]] = None   # (nodes N x 3, parents N)

    @property
    def success(self) -> bool:
        return len(self.path) > 0


class RRTPlanner:
    """RRT / RRT* over (x, y) with turn-in-place + straight-line edges."""

    def __init__(self, checker: FootprintCollisionChecker, config: Optional[PlannerConfig] = None):
        self.checker = checker
        self.config = config or PlannerConfig()
        if self.config.algorithm not in ('rrt', 'rrt_star'):
            raise ValueError(f"algorithm must be 'rrt' or 'rrt_star', got {self.config.algorithm!r}")
        self._rng = np.random.default_rng(self.config.seed)
        # Karaman & Frazzoli gamma* for d = 2 using the free area of the map
        # (the course code used the bounding box area and d = 2 for a 3D state).
        d = 2
        zeta_d = pi
        mu_free = max(checker.free_area(), 1e-6)
        gamma_star = 2.0 * (1.0 + 1.0 / d) ** (1.0 / d) * (mu_free / zeta_d) ** (1.0 / d)
        self._gamma = self.config.gamma_scale * gamma_star

    # Tree storage helpers
    def _reset(self, start: Pose, capacity: int) -> None:
        self._xy = np.zeros((capacity, 2))
        self._heading = np.zeros(capacity)       # arrival heading at each node
        self._parent = np.full(capacity, -1, dtype=np.int64)
        self._cost = np.full(capacity, np.inf)
        self._children: List[List[int]] = [[] for _ in range(capacity)]
        self._xy[0] = start[:2]
        self._heading[0] = start[2]
        self._cost[0] = 0.0
        self._n = 1
        self._goal: Optional[Pose] = None
        self._goal_parent = -1

    def _add_node(self, xy: np.ndarray, parent: int, cost: float) -> int:
        i = self._n
        self._xy[i] = xy
        self._heading[i] = atan2(xy[1] - self._xy[parent, 1], xy[0] - self._xy[parent, 0])
        self._parent[i] = parent
        self._cost[i] = cost
        self._children[parent].append(i)
        self._n += 1
        return i

    # Edge validity
    def _edge_free(self, parent: int, xy: np.ndarray) -> bool:
        """Turn at `parent` to face xy, then drive to xy."""
        px, py = self._xy[parent]
        heading = atan2(xy[1] - py, xy[0] - px)
        if self.checker.rotation_in_collision(px, py, self._heading[parent], heading):
            return False
        return not self.checker.segment_in_collision(px, py, xy[0], xy[1], heading)

    def _sample(self, goal: Pose) -> np.ndarray:
        if self._rng.random() < self.config.goal_bias:
            return np.array(goal[:2])
        (x0, x1), (y0, y1) = self.checker.x_bounds, self.checker.y_bounds
        # Reject samples whose centre is surely in collision (cheap).
        for _ in range(100):
            q = np.array([self._rng.uniform(x0, x1), self._rng.uniform(y0, y1)])
            if self.checker.clearance(q[0:1], q[1:2])[0] >= self.checker.inscribed_radius:
                return q
        return q

    def _steer(self, near: np.ndarray, target: np.ndarray) -> np.ndarray:
        delta = target - near
        dist = hypot(*delta)
        if dist <= self.config.step_size:
            return target.copy()
        return near + delta * (self.config.step_size / dist)

    def _goal_reachable(self, node: int, goal: Pose) -> bool:
        gx, gy, gyaw = goal
        if hypot(gx - self._xy[node, 0], gy - self._xy[node, 1]) < 1e-9:
            heading = self._heading[node]
        else:
            if not self._edge_free(node, np.array([gx, gy])):
                return False
            heading = atan2(gy - self._xy[node, 1], gx - self._xy[node, 0])
        if self.config.require_goal_yaw:
            return not self.checker.rotation_in_collision(gx, gy, heading, gyaw)
        return True

    def _update_subtree_costs(self, root: int) -> None:
        stack = [root]
        while stack:
            i = stack.pop()
            for c in self._children[i]:
                self._cost[c] = self._cost[i] + hypot(*(self._xy[c] - self._xy[i]))
                stack.append(c)

    def _rewire_ok(self, new: int, i: int) -> bool:
        """Can node i take `new` as its parent? i's arrival heading changes, so
        the in-place turn at i toward each of its children must be re-checked."""
        if not self._edge_free(new, self._xy[i]):
            return False
        new_heading = atan2(self._xy[i, 1] - self._xy[new, 1], self._xy[i, 0] - self._xy[new, 0])
        ix, iy = self._xy[i]
        for c in self._children[i]:
            out = atan2(self._xy[c, 1] - iy, self._xy[c, 0] - ix)
            if self.checker.rotation_in_collision(ix, iy, new_heading, out):
                return False
        if i == self._goal_parent:
            # The best solution leaves i toward the goal: re-check that as well.
            gx, gy, gyaw = self._goal
            if hypot(gx - ix, gy - iy) < 1e-9:
                out = new_heading
            else:
                out = atan2(gy - iy, gx - ix)
                if self.checker.rotation_in_collision(ix, iy, new_heading, out):
                    return False
            if self.config.require_goal_yaw and self.checker.rotation_in_collision(gx, gy, out, gyaw):
                return False
        return True

    # Planning
    def plan(self, start: Pose, goal: Pose) -> PlanResult:
        cfg = self.config
        t0 = time.monotonic()
        result = PlanResult()

        if self.checker.is_collision(*start):
            raise StartInCollision(f'start pose {start} is in collision')
        if self.checker.is_collision(*goal):
            raise GoalInCollision(f'goal pose {goal} is in collision')

        capacity = cfg.max_iterations + 2
        self._reset(start, capacity)
        self._goal = goal
        goal_xy = np.array(goal[:2])
        best_goal_parent = -1
        best_cost = np.inf
        no_improve = 0

        # Straight shot?
        if self._goal_reachable(0, goal):
            best_goal_parent = 0
            self._goal_parent = 0
            best_cost = hypot(*(goal_xy - self._xy[0]))
            result.first_solution_time = time.monotonic() - t0

        it = 0
        while best_goal_parent != 0 and it < cfg.max_iterations:
            it += 1
            if time.monotonic() - t0 > cfg.max_planning_time:
                break
            if cfg.algorithm == 'rrt' and best_goal_parent >= 0:
                break
            if cfg.algorithm == 'rrt_star' and best_goal_parent >= 0 and no_improve > cfg.patience:
                break
            no_improve += 1

            q_rand = self._sample(goal)
            d2 = np.sum((self._xy[:self._n] - q_rand) ** 2, axis=1)
            nearest = int(np.argmin(d2))
            q_new = self._steer(self._xy[nearest], q_rand)
            if hypot(*(q_new - self._xy[nearest])) < 1e-6:
                continue
            if not self._edge_free(nearest, q_new):
                continue

            parent = nearest
            new_cost = self._cost[nearest] + hypot(*(q_new - self._xy[nearest]))
            neighbors = np.empty(0, dtype=np.int64)

            if cfg.algorithm == 'rrt_star':
                n = max(self._n, 2)
                radius = min(self._gamma * sqrt(log(n) / n), cfg.max_rewire_radius)
                dists = np.sqrt(np.sum((self._xy[:self._n] - q_new) ** 2, axis=1))
                neighbors = np.nonzero(dists <= radius)[0]
                # Choose the cheapest collision-free parent.
                order = neighbors[np.argsort(self._cost[neighbors] + dists[neighbors])]
                for j in order:
                    c = self._cost[j] + dists[j]
                    if c >= new_cost:
                        break
                    if j == nearest:
                        continue
                    if self._edge_free(j, q_new):
                        parent, new_cost = int(j), c
                        break

            new = self._add_node(q_new, parent, new_cost)

            if cfg.algorithm == 'rrt_star':
                for j in neighbors:
                    j = int(j)
                    if j == parent or j == 0:
                        continue
                    c = new_cost + hypot(*(self._xy[j] - q_new))
                    if c + 1e-9 >= self._cost[j]:
                        continue
                    if not self._rewire_ok(new, j):
                        continue
                    old_parent = self._parent[j]
                    self._children[old_parent].remove(j)
                    self._parent[j] = new
                    self._children[new].append(j)
                    self._heading[j] = atan2(self._xy[j, 1] - q_new[1], self._xy[j, 0] - q_new[0])
                    self._cost[j] = c
                    self._update_subtree_costs(j)
                # Rewiring may have lowered the cost of the current best solution.
                if best_goal_parent >= 0:
                    best_cost = min(best_cost, self._cost[best_goal_parent] +
                                    hypot(*(goal_xy - self._xy[best_goal_parent])))

            dist_goal = hypot(*(goal_xy - q_new))
            if dist_goal <= cfg.goal_tolerance:
                cand = self._cost[new] + dist_goal
                if cand + 1e-9 < best_cost and self._goal_reachable(new, goal):
                    best_cost = cand
                    best_goal_parent = new
                    self._goal_parent = new
                    no_improve = 0
                    if result.first_solution_time is None:
                        result.first_solution_time = time.monotonic() - t0

        result.iterations = it
        result.nodes = self._n
        result.tree = (np.column_stack([self._xy[:self._n], self._heading[:self._n]]).copy(),
                       self._parent[:self._n].copy())

        if best_goal_parent >= 0:
            raw = self._reconstruct(best_goal_parent, goal)
            result.raw_path = raw
            result.path = self.shortcut(raw, goal)
            result.cost = path_length(result.path)
        result.planning_time = time.monotonic() - t0
        return result

    def _reconstruct(self, goal_parent: int, goal: Pose) -> List[Pose]:
        chain = []
        i = goal_parent
        while i >= 0:
            chain.append(i)
            i = int(self._parent[i])
        chain.reverse()
        points = [tuple(self._xy[i]) for i in chain]
        if hypot(goal[0] - points[-1][0], goal[1] - points[-1][1]) > 1e-9:
            points.append((goal[0], goal[1]))
        return poses_from_points(points, start_yaw=self._heading[0], goal_yaw=goal[2])

    # Post-processing
    def path_valid(self, path: List[Pose]) -> bool:
        """Full re-check of a waypoint path: turn at each waypoint, then drive."""
        if not path:
            return False
        heading = path[0][2]
        for (x0, y0, _), (x1, y1, _) in zip(path[:-1], path[1:]):
            seg = atan2(y1 - y0, x1 - x0)
            if self.checker.rotation_in_collision(x0, y0, heading, seg):
                return False
            if self.checker.segment_in_collision(x0, y0, x1, y1, seg):
                return False
            heading = seg
        if self.config.require_goal_yaw:
            x, y, yaw = path[-1]
            return not self.checker.rotation_in_collision(x, y, heading, yaw)
        return True

    def shortcut(self, path: List[Pose], goal: Pose) -> List[Pose]:
        """Random shortcutting (shortcutPath.m) that keeps the whole path executable."""
        points = [(p[0], p[1]) for p in path]
        start_yaw = path[0][2]
        for _ in range(self.config.shortcut_attempts):
            n = len(points)
            if n < 3:
                break
            i = int(self._rng.integers(0, n - 2))
            j = int(self._rng.integers(i + 2, n))
            candidate = points[:i + 1] + points[j:]
            if self.path_valid(poses_from_points(candidate, start_yaw, goal[2])):
                points = candidate
        return poses_from_points(points, start_yaw, goal[2])


class StartInCollision(RuntimeError):
    pass


class GoalInCollision(RuntimeError):
    pass


def poses_from_points(points: List[Tuple[float, float]], start_yaw: float, goal_yaw: float) -> List[Pose]:
    """Attach headings to waypoints: each one faces the next, the last uses goal_yaw.
    The first waypoint (the start) keeps start_yaw so path_valid sees the initial turn."""
    poses: List[Pose] = []
    for k, (x, y) in enumerate(points):
        if k == 0:
            yaw = start_yaw
        elif k + 1 < len(points):
            yaw = atan2(points[k + 1][1] - y, points[k + 1][0] - x)
        else:
            yaw = goal_yaw
        poses.append((float(x), float(y), float(yaw)))
    return poses


def path_length(path: List[Pose]) -> float:
    return float(sum(hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path[:-1], path[1:])))


def densify(path: List[Pose], spacing: float) -> List[Pose]:
    """Insert waypoints so consecutive ones are at most `spacing` apart.
    Inserted points face along their segment, matching the straight-drive edge."""
    if spacing <= 0.0 or len(path) < 2:
        return list(path)
    out: List[Pose] = []
    for (x0, y0, yaw0), (x1, y1, _) in zip(path[:-1], path[1:]):
        seg = atan2(y1 - y0, x1 - x0)
        n = max(1, int(ceil(hypot(x1 - x0, y1 - y0) / spacing)))
        out.append((x0, y0, seg if out else yaw0))
        for k in range(1, n):
            a = k / n
            out.append((x0 + a * (x1 - x0), y0 + a * (y1 - y0), seg))
    out.append(path[-1])
    return out
