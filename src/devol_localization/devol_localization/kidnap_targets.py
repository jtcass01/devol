"""Collision-checked kidnap targets (ROS-free).

The kidnapper teleports the robot in Gazebo. A target inside an obstacle makes the physics engine
shove the robot out somewhere unpredictable, and the planner cannot plan from a start in collision.
These helpers check a target with the RRT planner's own 3D body check (body boxes against the
world's static point cloud, voxelised like octomap_server does) and, for random targets, also check
that the planner can reach the next goal from there, which keeps targets inside the building.
"""

from math import hypot, pi
from typing import Optional, Sequence, Tuple

import numpy as np

from devol_sim.rrt_planner import (
    GoalInCollision,
    PlannerConfig,
    RRTPlanner,
    StartInCollision,
    body_checker_from_voxels,
)

__author__ = 'Jacob Taylor Cassady'
__email__ = 'jcassad1@jh.edu'

Pose = Tuple[float, float, float]


def voxels_from_pcd(
    path: str, resolution: float = 0.05, min_z: float = 0.1, max_z: float = 5.0
) -> np.ndarray:
    """Occupied voxel centres of a static .pcd (ASCII, x y z), filtered to min_z..max_z like
    octomap_server's occupancy_min_z / occupancy_max_z."""
    pts = np.loadtxt(path, skiprows=11, usecols=(0, 1, 2))
    keys = np.unique(np.floor(pts / resolution).astype(int), axis=0)
    centers = (keys + 0.5) * resolution
    keep = (centers[:, 2] + resolution / 2 > min_z) & (centers[:, 2] - resolution / 2 < max_z)
    return centers[keep]


def world_checker(pcd_path: str, resolution: float = 0.05, clearance: float = 0.15):
    """The planner's 3D body checker for a world, with `clearance` m of padding around each box."""
    return body_checker_from_voxels(
        voxels_from_pcd(pcd_path, resolution), resolution, resolution, padding=clearance
    )


def is_free(checker, pose: Pose) -> bool:
    return not checker.is_collision(*pose)


def is_reachable(
    checker, start: Pose, goal: Pose, seed: int = 0, max_planning_time: float = 2.0
) -> bool:
    """Can the planner find a path from start to goal?"""
    config = PlannerConfig(algorithm='rrt', seed=seed, max_planning_time=max_planning_time)
    try:
        return RRTPlanner(checker, config).plan(start, goal).success
    except (StartInCollision, GoalInCollision):
        return False


def sample_target(
    checker,
    rng: np.random.Generator,
    robot_xy: Sequence[float],
    min_distance: float,
    reach_goal: Optional[Pose] = None,
    bounds_margin: float = 1.0,
    max_tries: int = 2000,
    max_reach_checks: int = 10,
) -> Optional[Pose]:
    """A random collision-free pose at least min_distance m from robot_xy (and from reach_goal, so
    the kidnap does not drop the robot on the goal it is driving to), from which the planner can
    reach reach_goal. None if nothing qualifies within the tries."""
    (x0, x1), (y0, y1) = checker.x_bounds, checker.y_bounds
    reach_checks = 0
    for _ in range(max_tries):
        pose = (
            float(rng.uniform(x0 + bounds_margin, x1 - bounds_margin)),
            float(rng.uniform(y0 + bounds_margin, y1 - bounds_margin)),
            float(rng.uniform(-pi, pi)),
        )
        if hypot(pose[0] - robot_xy[0], pose[1] - robot_xy[1]) < min_distance:
            continue
        if (
            reach_goal is not None
            and hypot(pose[0] - reach_goal[0], pose[1] - reach_goal[1]) < min_distance
        ):
            continue
        if not is_free(checker, pose):
            continue
        if reach_goal is not None:
            if reach_checks >= max_reach_checks:
                return None
            reach_checks += 1
            if not is_reachable(checker, pose, reach_goal, seed=int(rng.integers(1 << 31))):
                continue
        return pose
    return None
