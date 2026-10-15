import numpy as np
import pytest

from dist_project.ekf import UnicycleEKF, range_std, wrap_angle

ANCHORS = np.array([[-4, -4, 3], [-4, 4, 3], [0, -4, 3], [4, -4, 3], [4, 4, 3]], dtype=float)


def simulate_ranges(rng, pos, ekf):
    true = np.linalg.norm(ANCHORS - np.array([pos[0], pos[1], 0.0]), axis=1)
    return true + rng.normal(0.0, range_std(true, ekf.range_rel_std, ekf.range_floor_std))


def test_wrap_angle():
    assert wrap_angle(3 * np.pi / 2) == pytest.approx(-np.pi / 2)
    assert wrap_angle(-3 * np.pi / 2) == pytest.approx(np.pi / 2)
    assert wrap_angle(0.3) == pytest.approx(0.3)


def test_predict_straight_line_and_turn():
    ekf = UnicycleEKF(mu0=[0, 0, np.pi / 2])
    ekf.predict(v=1.0, w=0.0, dt=0.5)
    np.testing.assert_allclose(ekf.mu, [0.0, 0.5, np.pi / 2], atol=1e-12)
    ekf.predict(v=0.0, w=np.pi, dt=1.0)
    assert ekf.mu[2] == pytest.approx(-np.pi / 2)


def test_predict_grows_uncertainty():
    ekf = UnicycleEKF(sigma0=np.eye(3) * 1e-4)
    before = np.trace(ekf.sigma)
    ekf.predict(v=0.5, w=0.2, dt=0.1)
    assert np.trace(ekf.sigma) > before


def test_update_is_deterministic_and_symmetric():
    rng = np.random.default_rng(0)
    a = UnicycleEKF(mu0=[0.5, -0.5, 0.0])
    b = UnicycleEKF(mu0=[0.5, -0.5, 0.0])
    z = simulate_ranges(rng, (1.0, 1.0), a)
    a.update(z, ANCHORS)
    b.update(z, ANCHORS)
    np.testing.assert_array_equal(a.mu, b.mu)
    np.testing.assert_allclose(a.sigma, a.sigma.T)
    assert np.all(np.linalg.eigvalsh(a.sigma) > 0)


def test_update_reduces_position_uncertainty():
    rng = np.random.default_rng(1)
    ekf = UnicycleEKF()
    before = ekf.std[:2].copy()
    ekf.update(simulate_ranges(rng, (0.0, 0.0), ekf), ANCHORS)
    assert np.all(ekf.std[:2] < before)
    # The distances don't carry information on theta
    assert ekf.std[2] == pytest.approx(1.0)


def test_empty_update():
    ekf = UnicycleEKF(mu0=[1, 2, 0.3])
    ekf.update([], np.empty((0, 3)))
    np.testing.assert_array_equal(ekf.mu, [1, 2, 0.3])


def test_tracking_circular_trajectory():
    rng = np.random.default_rng(42)
    dt, v, w = 0.1, 0.3, 0.15
    truth = np.array([1.0, 0.0, np.pi / 2])
    ekf = UnicycleEKF(mu0=[0.0, 0.0, np.pi / 2 + 0.3])

    errors, nees = [], []
    for k in range(600):
        truth = truth + np.array([v * np.cos(truth[2]) * dt, v * np.sin(truth[2]) * dt, w * dt])
        ekf.predict(v, w, dt)
        ekf.update(simulate_ranges(rng, truth, ekf), ANCHORS)
        if k > 100:
            e = ekf.mu[:2] - truth[:2]
            errors.append(np.linalg.norm(e))
            nees.append(e @ np.linalg.inv(ekf.sigma[:2, :2]) @ e)

    assert np.sqrt(np.mean(np.square(errors))) < 0.15
    # The covariance must not be overconfident, the mean NEES of a 2D error is around 2
    assert np.mean(nees) < 4.0


def test_heading_frozen_when_still():
    rng = np.random.default_rng(3)
    ekf = UnicycleEKF(mu0=[0.5, 0.5, 0.4])
    # Correlation between x and theta as after a straight motion
    ekf.sigma[0, 2] = ekf.sigma[2, 0] = 0.3
    for _ in range(50):
        ekf.update(simulate_ranges(rng, (0.5, 0.5), ekf), ANCHORS, update_heading=False)
    assert ekf.mu[2] == pytest.approx(0.4)
    np.testing.assert_allclose(ekf.sigma, ekf.sigma.T)
    assert np.all(np.linalg.eigvalsh(ekf.sigma) > 0)
