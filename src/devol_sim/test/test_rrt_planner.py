"""Offline tests for the RRT / RRT* planner and its footprint collision check."""

import os
from math import hypot, pi

import numpy as np
import pytest

from devol_sim.rrt_planner import (
    DEFAULT_BODY,
    FootprintCollisionChecker,
    PlannerConfig,
    RRTPlanner,
    StartInCollision,
    body_checker_from_voxels,
    densify,
    layer_grids_from_voxels,
    path_length,
)

RES = 0.05
FACTORY_PCD = os.path.join(
    os.path.dirname(__file__), '..', '..', 'devol_gazebo', 'worlds', 'factory', 'static_world.pcd'
)


def empty_grid(width_m, height_m):
    return np.zeros((int(round(height_m / RES)), int(round(width_m / RES))), dtype=np.int8)


def fill(grid, x0, y0, x1, y1, value=100, origin=(0.0, 0.0)):
    c0, c1 = int((x0 - origin[0]) / RES), int(np.ceil((x1 - origin[0]) / RES))
    r0, r1 = int((y0 - origin[1]) / RES), int(np.ceil((y1 - origin[1]) / RES))
    grid[r0:r1, c0:c1] = value


def projected_grid_from_pcd(path, resolution=RES, min_z=0.1, max_z=5.0):
    """Approximates octomap_server's projected_map for a static cloud:
    any point between min_z and max_z marks its column occupied."""
    pts = np.loadtxt(path, skiprows=11)
    origin = (
        np.floor(pts[:, 0].min() / resolution) * resolution,
        np.floor(pts[:, 1].min() / resolution) * resolution,
    )
    cols = np.floor((pts[:, 0] - origin[0]) / resolution).astype(int)
    rows = np.floor((pts[:, 1] - origin[1]) / resolution).astype(int)
    grid = np.zeros((rows.max() + 1, cols.max() + 1), dtype=np.int8)
    keep = (pts[:, 2] >= min_z) & (pts[:, 2] <= max_z)
    grid[rows[keep], cols[keep]] = 100
    return grid, origin


def voxels_from_pcd(path, resolution=RES, min_z=0.1, max_z=5.0):
    """Approximates octomap_server's occupied leaves for a static cloud
    (finest leaves only, filtered to occupancy_min_z..occupancy_max_z)."""
    pts = np.loadtxt(path, skiprows=11)
    keys = np.unique(np.floor(pts / resolution).astype(int), axis=0)
    centers = (keys + 0.5) * resolution
    keep = (centers[:, 2] + resolution / 2 > min_z) & (centers[:, 2] - resolution / 2 < max_z)
    return centers[keep]


def exact_3d_collision(voxel_keys, body, pose, padding=0.05, step=RES / 4):
    """Independent reference: sample each padded 3D body box densely and look the
    points up in a dense 3D array of finest-voxel keys."""
    occ, k0 = voxel_keys
    x, y, th = pose
    c, s = np.cos(th), np.sin(th)
    for b in body:
        xs = np.arange(b.x_min - padding, b.x_max + padding + 1e-9, step)
        ys = np.arange(b.y_min - padding, b.y_max + padding + 1e-9, step)
        zs = np.arange(b.z_min + 1e-6, b.z_max + padding, step)
        bx, by, bz = np.meshgrid(xs, ys, zs, indexing='ij')
        wx, wy = x + c * bx - s * by, y + s * bx + c * by
        keys = np.floor(np.stack([wx.ravel(), wy.ravel(), bz.ravel()], 1) / RES).astype(int) - k0
        inside = np.all((keys >= 0) & (keys < occ.shape), axis=1)
        if occ[tuple(keys[inside].T)].any():
            return True
    return False


def voxel_key_array(voxels):
    keys = np.floor(voxels / RES).astype(int)
    k0 = keys.min(0)
    occ = np.zeros(keys.max(0) - k0 + 1, dtype=bool)
    occ[tuple((keys - k0).T)] = True
    return occ, k0


def dense_path_poses(checker, path):
    """Every pose the robot passes through: turn at each waypoint, then drive."""
    poses = []
    heading = path[0][2]
    for (x0, y0, _), (x1, y1, _) in zip(path[:-1], path[1:]):
        seg = np.arctan2(y1 - y0, x1 - x0)
        poses.append(checker.rotation_poses(x0, y0, heading, seg))
        poses.append(checker.segment_poses(x0, y0, x1, y1, seg))
        heading = seg
    poses.append(checker.rotation_poses(path[-1][0], path[-1][1], heading, path[-1][2]))
    return np.vstack(poses)


# Collision checker
def test_footprint_is_orientation_dependent():
    # Obstacle 0.45 m to the side of the robot centre: clear when the robot's
    # long axis points at it sideways (half-width 0.385 with padding), hit when
    # the long axis points at it (half-length 0.545).
    grid = empty_grid(4, 4)
    fill(grid, 2.45, 1.9, 2.5, 2.1)
    checker = FootprintCollisionChecker(grid, RES, (0.0, 0.0))
    assert not checker.is_collision(2.0, 2.0, pi / 2)
    assert checker.is_collision(2.0, 2.0, 0.0)


def test_footprint_detects_cell_in_interior_and_corner():
    grid = empty_grid(4, 4)
    checker_free = FootprintCollisionChecker(grid.copy(), RES, (0.0, 0.0))
    assert not checker_free.is_collision(2.0, 2.0, 0.3)
    fill(grid, 2.0, 2.0, 2.05, 2.05)  # single cell under the robot
    assert FootprintCollisionChecker(grid, RES, (0.0, 0.0)).is_collision(2.0, 2.0, 0.3)

    grid = empty_grid(4, 4)
    # Single cell just inside the padded corner at yaw 0 (corner at +0.545, +0.385).
    fill(grid, 2.50, 2.35, 2.55, 2.40)
    checker = FootprintCollisionChecker(grid, RES, (0.0, 0.0))
    assert checker.is_collision(2.0, 2.0, 0.0)
    assert not checker.is_collision(2.0, 2.0, pi / 2)


def test_footprint_off_map_and_unknown():
    grid = empty_grid(4, 4)
    checker = FootprintCollisionChecker(grid, RES, (0.0, 0.0))
    assert checker.is_collision(0.3, 2.0, 0.0)  # rear hangs off the map
    fill(grid, 1.9, 1.9, 2.1, 2.1, value=-1)
    assert FootprintCollisionChecker(grid, RES, (0.0, 0.0)).is_collision(2.0, 2.0, 0.0)
    assert not FootprintCollisionChecker(
        grid, RES, (0.0, 0.0), unknown_is_occupied=False
    ).is_collision(2.0, 2.0, 0.0)


def test_fast_paths_agree_with_exact_check():
    rng = np.random.default_rng(0)
    grid = (rng.random((80, 80)) < 0.01).astype(np.int8) * 100
    checker = FootprintCollisionChecker(grid, RES, (0.0, 0.0))
    poses = np.column_stack(
        [rng.uniform(0, 4, 2000), rng.uniform(0, 4, 2000), rng.uniform(-pi, pi, 2000)]
    )
    bx, by = checker._body_points
    for x, y, th in poses:
        c, s = np.cos(th), np.sin(th)
        rows, cols, inside = checker.world_to_cells(x + c * bx - s * by, y + s * bx + c * by)
        exact = (not inside.all()) or bool(checker.occupied[rows, cols].any())
        assert checker.is_collision(x, y, th) == exact


# Planner
def corridor_world():
    """8 x 6 m room, wall at x = 4 with a 0.9 m wide door. The robot (0.67 m
    wide plus padding) only fits through the door driving lengthwise."""
    grid = empty_grid(8, 6)
    fill(grid, 0, 0, 8, 0.1)
    fill(grid, 0, 5.9, 8, 6)
    fill(grid, 0, 0, 0.1, 6)
    fill(grid, 7.9, 0, 8, 6)
    fill(grid, 3.9, 0, 4.1, 2.55)
    fill(grid, 3.9, 3.45, 4.1, 6)
    return grid


@pytest.mark.parametrize('algorithm', ['rrt', 'rrt_star'])
def test_plans_through_door(algorithm):
    checker = FootprintCollisionChecker(corridor_world(), RES, (0.0, 0.0))
    planner = RRTPlanner(
        checker, PlannerConfig(algorithm=algorithm, seed=1, max_planning_time=10.0)
    )
    start, goal = (1.5, 1.5, pi / 2), (6.5, 4.5, 0.0)
    result = planner.plan(start, goal)
    assert result.success
    assert result.path[0][:2] == start[:2] and result.path[-1] == goal
    assert planner.path_valid(result.path) and planner.path_valid(result.raw_path)
    assert not checker.poses_in_collision(dense_path_poses(checker, result.path))
    # Densified waypoints lie on the same segments, so they stay valid too.
    assert planner.path_valid(densify(result.path, 0.5))


def test_door_too_narrow_sideways_is_detected():
    checker = FootprintCollisionChecker(corridor_world(), RES, (0.0, 0.0))
    # Robot sitting in the doorway facing along the wall (sideways through the door).
    assert checker.is_collision(4.0, 3.0, pi / 2)
    assert not checker.is_collision(4.0, 3.0, 0.0)


def test_rrt_star_not_worse_than_rrt():
    checker = FootprintCollisionChecker(corridor_world(), RES, (0.0, 0.0))
    start, goal = (1.5, 1.5, 0.0), (6.5, 4.5, 0.0)
    rrt, star = [], []
    for seed in range(5):
        rrt.append(
            RRTPlanner(
                checker,
                PlannerConfig(
                    algorithm='rrt', seed=seed, shortcut_attempts=0, max_planning_time=10.0
                ),
            )
            .plan(start, goal)
            .cost
        )
        star.append(
            RRTPlanner(
                checker,
                PlannerConfig(
                    algorithm='rrt_star', seed=seed, shortcut_attempts=0, max_planning_time=10.0
                ),
            )
            .plan(start, goal)
            .cost
        )
    assert np.mean(star) <= np.mean(rrt)


def test_start_in_collision_raises():
    checker = FootprintCollisionChecker(corridor_world(), RES, (0.0, 0.0))
    with pytest.raises(StartInCollision):
        RRTPlanner(checker).plan((4.0, 1.0, 0.0), (6.5, 4.5, 0.0))


def test_unknown_cells_under_start_are_freed():
    # octomap_server leaves the voxel at the cloud's sensor origin unknown,
    # which is exactly where the robot spawns.
    grid = corridor_world()
    fill(grid, 1.45, 1.45, 1.6, 1.6, value=-1)
    fill(grid, 6.0, 1.0, 6.2, 1.2, value=-1)  # unknown pocket elsewhere stays blocked
    start, goal = (1.5, 1.5, 0.0), (6.5, 4.5, 0.0)

    checker = FootprintCollisionChecker(grid, RES, (0.0, 0.0))
    assert checker.is_collision(*start)
    with pytest.raises(StartInCollision):
        RRTPlanner(checker, PlannerConfig(free_unknown_at_start=False)).plan(start, goal)

    checker = FootprintCollisionChecker(grid, RES, (0.0, 0.0))
    result = RRTPlanner(checker, PlannerConfig(seed=0, max_planning_time=10.0)).plan(start, goal)
    assert result.freed_start_cells == (grid[20:40, 20:40] < 0).sum() > 0
    assert result.success
    assert checker.is_collision(6.1, 1.1, 0.0)
    # Occupied cells under the start are not freed.
    grid[30, 30] = 100
    with pytest.raises(StartInCollision):
        RRTPlanner(FootprintCollisionChecker(grid, RES, (0.0, 0.0))).plan(start, goal)


def test_straight_shot_in_open_space():
    checker = FootprintCollisionChecker(empty_grid(6, 6), RES, (0.0, 0.0))
    result = RRTPlanner(checker, PlannerConfig(seed=0, goal_approach_distance=0.0)).plan(
        (1.0, 1.0, 0.0), (5.0, 5.0, pi)
    )
    assert len(result.path) == 2
    assert result.cost == pytest.approx(hypot(4.0, 4.0))


@pytest.mark.skipif(not os.path.exists(FACTORY_PCD), reason='factory static_world.pcd not found')
def test_factory_world_goals():
    grid, origin = projected_grid_from_pcd(FACTORY_PCD)
    checker = FootprintCollisionChecker(grid, RES, origin)
    # Spawn pose and goals from devol_gazebo/worlds/factory/poses.csv, in visiting order.
    waypoints = [(0.0, 0.0, 0.0), (-4.7, 9.2, 3.14159), (5.45, 2.03, 0.0), (1.87, -8.06, -1.5708)]
    for start, goal in zip(waypoints[:-1], waypoints[1:]):
        planner = RRTPlanner(checker, PlannerConfig(seed=0, max_planning_time=10.0))
        result = planner.plan(start, goal)
        assert result.success, f'no path {start} -> {goal}'
        assert not checker.poses_in_collision(dense_path_poses(checker, result.path))
        assert path_length(result.path) >= hypot(goal[0] - start[0], goal[1] - start[1]) - 1e-6


# 3D (octomap voxels)
def box_voxels(x0, x1, y0, y1, z0, z1, res=RES):
    xs = np.arange(x0 + res / 2, x1, res)
    ys = np.arange(y0 + res / 2, y1, res)
    zs = np.arange(z0 + res / 2, z1, res)
    return np.stack(np.meshgrid(xs, ys, zs, indexing='ij'), -1).reshape(-1, 3)


def walled_room_voxels(width=8.0, height=6.0, wall_top=2.0):
    return np.vstack(
        [
            box_voxels(0, width, 0, 0.1, 0.1, wall_top),
            box_voxels(0, width, height - 0.1, height, 0.1, wall_top),
            box_voxels(0, 0.1, 0, height, 0.1, wall_top),
            box_voxels(width - 0.1, width, 0, height, 0.1, wall_top),
        ]
    )


def test_pruned_leaves_fill_all_their_cells():
    # One 0.2 m leaf (a pruned 4x4x4 block) and one finest leaf.
    centers = np.array([[1.1, 1.1, 0.3], [2.025, 2.025, 1.025]])
    grids, origin = layer_grids_from_voxels(centers, [0.2, 0.05], RES, [(0.05, 0.4), (0.4, 1.5)])
    low, high = grids
    assert origin == (1.0, 1.0)
    assert (low == 100).sum() == 16 and (high == 100).sum() == 1  # the big leaf tops out at 0.4 m
    assert low[0:4, 0:4].all() and high[20, 20] == 100 and low[20, 20] == 0


def test_3d_drives_under_overhang_but_not_into_low_obstacle():
    room = walled_room_voxels()
    beam = box_voxels(3.9, 4.1, 0.1, 5.9, 1.8, 2.0)  # overhead beam across the room at 1.8-2.0 m
    checker = body_checker_from_voxels(np.vstack([room, beam]), RES, RES, margin=0.0)
    assert not checker.is_collision(4.0, 3.0, 0.0)  # robot (<= 1.5 m tall) fits under it
    planner = RRTPlanner(checker, PlannerConfig(seed=0, max_planning_time=10.0))
    result = planner.plan((1.5, 3.0, 0.0), (6.5, 3.0, 0.0))
    assert result.success and result.cost == pytest.approx(5.0)  # straight across, under the beam

    # The same beam lowered to table height blocks the arm, and a 20 cm curb blocks the base.
    for obstacle in (
        box_voxels(3.9, 4.1, 0.1, 5.9, 0.6, 0.7),
        box_voxels(3.9, 4.1, 0.1, 5.9, 0.1, 0.2),
    ):
        checker = body_checker_from_voxels(np.vstack([room, obstacle]), RES, RES, margin=0.0)
        assert checker.is_collision(4.0, 3.0, 0.0)
        result = RRTPlanner(checker, PlannerConfig(seed=0, max_planning_time=1.0)).plan(
            (1.5, 3.0, 0.0), (6.5, 3.0, 0.0)
        )
        assert not result.success


def test_arm_box_overhangs_under_a_table():
    # A table top at 0.6-0.7 m reaching 0.3 m in from the wall: the base (0.40 m
    # tall) fits under it but the arm box does not.
    room = walled_room_voxels()
    table = box_voxels(0.1, 8.0, 0.1, 0.4, 0.6, 0.7)
    checker = body_checker_from_voxels(np.vstack([room, table]), RES, RES, margin=0.0)
    base_only = body_checker_from_voxels(
        np.vstack([room, table]), RES, RES, body=DEFAULT_BODY[:1], margin=0.0
    )
    pose = (4.0, 0.57, 0.0)  # padded base reaches 0.185 m, under the table
    assert not base_only.is_collision(*pose)
    assert checker.is_collision(*pose)  # the arm box (padded to 0.27 m) hits the table top
    assert not checker.is_collision(4.0, 0.75, 0.0)


@pytest.mark.skipif(not os.path.exists(FACTORY_PCD), reason='factory static_world.pcd not found')
def test_factory_3d_check_matches_exact_voxel_test():
    voxels = voxels_from_pcd(FACTORY_PCD)
    checker = body_checker_from_voxels(voxels, RES, RES)
    voxel_keys = voxel_key_array(voxels)
    rng = np.random.default_rng(1)
    (x0, x1), (y0, y1) = checker.x_bounds, checker.y_bounds
    poses = np.column_stack(
        [
            rng.uniform(x0 + 1.5, x1 - 1.5, 300),
            rng.uniform(y0 + 1.5, y1 - 1.5, 300),
            rng.uniform(-pi, pi, 300),
        ]
    )
    hits = 0
    for pose in poses:
        expected = exact_3d_collision(voxel_keys, DEFAULT_BODY, pose)
        hits += expected
        assert checker.is_collision(*pose) == expected, pose
    assert 0 < hits < len(poses)


@pytest.mark.skipif(not os.path.exists(FACTORY_PCD), reason='factory static_world.pcd not found')
def test_factory_world_goals_3d():
    voxels = voxels_from_pcd(FACTORY_PCD)
    checker = body_checker_from_voxels(voxels, RES, RES)
    waypoints = [(0.0, 0.0, 0.0), (-4.7, 9.2, 3.14159), (5.45, 2.03, 0.0), (1.87, -8.06, -1.5708)]
    for start, goal in zip(waypoints[:-1], waypoints[1:]):
        planner = RRTPlanner(checker, PlannerConfig(seed=0, max_planning_time=10.0))
        result = planner.plan(start, goal)
        assert result.success, f'no path {start} -> {goal}'
        assert not checker.poses_in_collision(dense_path_poses(checker, result.path))


def test_path_ends_with_straight_run_along_goal_heading():
    checker = FootprintCollisionChecker(corridor_world(), RES, (0.0, 0.0))
    goal = (6.5, 4.5, pi / 2)
    result = RRTPlanner(checker, PlannerConfig(seed=0, max_planning_time=10.0)).plan(
        (1.5, 1.5, 0.0), goal
    )
    assert result.success and result.path[-1] == goal
    (x0, y0, _), (x1, y1, _) = result.path[-2:]
    assert np.arctan2(y1 - y0, x1 - x0) == pytest.approx(goal[2], abs=1e-9)
    assert hypot(x1 - x0, y1 - y0) == pytest.approx(1.0)
    assert RRTPlanner(checker).path_valid(result.path)

    # Lead-in shortened when the full 1 m would hit the wall behind the goal.
    goal = (6.5, 1.1, -pi / 2)
    result = RRTPlanner(checker, PlannerConfig(seed=0, max_planning_time=10.0)).plan(
        (1.5, 4.5, 0.0), goal
    )
    (x0, y0, _), (x1, y1, _) = result.path[-2:]
    assert np.arctan2(y1 - y0, x1 - x0) == pytest.approx(goal[2], abs=1e-9)
    assert not checker.poses_in_collision(dense_path_poses(checker, result.path))
