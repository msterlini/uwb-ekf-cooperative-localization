import numpy as np
import pytest

from dist_project.triangulation import VERTICAL_TILT, estimate_height, intersect_bearings

TARGET = np.array([1.0, 2.0])


def robots_pointing_at(target, positions, sigmas=(0.05, 0.05, 0.02)):
    rows = []
    for x, y in positions:
        theta = np.arctan2(target[1] - y, target[0] - x)
        rows.append([x, y, theta, *sigmas])
    return np.array(rows)


def test_exact_intersection():
    robots = robots_pointing_at(TARGET, [(-3, -3), (3, -3), (-3, 3), (3, 3)])
    p, cov = intersect_bearings(robots)
    np.testing.assert_allclose(p, TARGET, atol=1e-9)
    np.testing.assert_allclose(cov, cov.T)


def test_vertical_lines():
    # theta = 90 degrees, where the slope tan(theta) would diverge
    robots = robots_pointing_at(TARGET, [(1.0, -3.0), (4.0, 2.0)])
    assert robots[0, 2] == pytest.approx(np.pi / 2)
    p, _ = intersect_bearings(robots)
    np.testing.assert_allclose(p, TARGET, atol=1e-9)


def test_single_or_parallel_lines():
    assert intersect_bearings([[0, 0, 0, 0.1, 0.1, 0.1]]) == (None, None)
    parallel = [[0, 0, 0.0, 0.1, 0.1, 0.1], [0, 1, 0.0, 0.1, 0.1, 0.1]]
    assert intersect_bearings(parallel) == (None, None)


def test_uncertain_robot_has_less_weight():
    robots = robots_pointing_at(TARGET, [(-3, -3), (3, -3), (-3, 3)])
    robots[2, 2] += 0.2
    robots[2, 5] = 1.0
    p, _ = intersect_bearings(robots)
    assert np.linalg.norm(p - TARGET) < 0.05


def test_noisy_robots():
    rng = np.random.default_rng(0)
    positions = [(-3, -3), (3, -3), (-3, 3), (3, 3), (0, -4)]
    errors = []
    for _ in range(300):
        robots = robots_pointing_at(TARGET, positions)
        robots[:, 0] += rng.normal(0, 0.05, len(positions))
        robots[:, 1] += rng.normal(0, 0.05, len(positions))
        robots[:, 2] += rng.normal(0, 0.02, len(positions))
        p, _ = intersect_bearings(robots)
        errors.append(np.linalg.norm(p - TARGET))
    assert np.sqrt(np.mean(np.square(errors))) < 0.15


def test_height_from_multiple_robots():
    robots = robots_pointing_at(TARGET, [(-3, -3), (3, -3), (-3, 3)])
    z = 0.8
    distance = np.hypot(TARGET[0] - robots[:, 0], TARGET[1] - robots[:, 1])
    target_heights = np.arctan(z / distance) / VERTICAL_TILT
    h, sigma = estimate_height(TARGET, robots, target_heights)
    assert h == pytest.approx(z)
    assert sigma > 0
