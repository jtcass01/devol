"""Offline tests for the collision-checked kidnap targets on the factory world."""

import os
from math import hypot

import numpy as np
import pytest

from devol_localization.kidnap_targets import (
    is_free,
    is_reachable,
    sample_target,
    voxels_from_pcd,
    world_checker,
)

FACTORY_PCD = os.path.join(
    os.path.dirname(__file__), '..', '..', 'devol_gazebo', 'worlds', 'factory', 'static_world.pcd'
)
GOALS = [(-4.7, 9.2, 3.14159), (5.45, 2.03, 0.0), (1.87, -8.06, -1.5708)]
pytestmark = pytest.mark.skipif(
    not os.path.exists(FACTORY_PCD), reason='factory static_world.pcd not found'
)


@pytest.fixture(scope='module')
def checker():
    return world_checker(FACTORY_PCD)


def test_route_goals_are_free(checker):
    # Test case 2 teleports onto Goal 2; the check must not block it.
    for goal in [(0.0, 0.0, 0.0)] + GOALS:
        assert is_free(checker, goal), goal


def test_obstacles_are_not_free(checker):
    voxels = voxels_from_pcd(FACTORY_PCD)
    low = voxels[(voxels[:, 2] > 0.1) & (voxels[:, 2] < 0.4)]
    rng = np.random.default_rng(0)
    for x, y, _ in low[rng.choice(len(low), 20, replace=False)]:
        assert not is_free(checker, (x, y, 0.0))


def test_random_targets_are_free_far_and_reachable(checker):
    robot = (-2.0, 7.0)  # on the way from Goal 1 to Goal 2
    goal2 = GOALS[1]
    targets = []
    for seed in range(5):
        target = sample_target(checker, np.random.default_rng(seed), robot, 3.0, reach_goal=goal2)
        assert target is not None, seed
        assert is_free(checker, target)
        assert hypot(target[0] - robot[0], target[1] - robot[1]) >= 3.0
        assert hypot(target[0] - goal2[0], target[1] - goal2[1]) >= 3.0
        assert is_reachable(checker, target, goal2)
        targets.append(target)
    assert len(set(targets)) == 5
    assert (
        sample_target(checker, np.random.default_rng(0), robot, 3.0, reach_goal=goal2)
        == targets[0]
    )
