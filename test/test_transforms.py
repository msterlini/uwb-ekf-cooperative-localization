import numpy as np
import pytest

from dist_project.transforms import quaternion_from_yaw, yaw_from_quaternion


@pytest.mark.parametrize("yaw", [0.0, 0.5, -1.2, np.pi / 2, 3.0, -3.0])
def test_round_trip(yaw):
    assert yaw_from_quaternion(*quaternion_from_yaw(yaw)) == pytest.approx(yaw)


def test_unit_quaternion():
    q = np.array(quaternion_from_yaw(0.7))
    assert np.linalg.norm(q) == pytest.approx(1.0)


def test_known_values():
    # 90 degrees around z
    assert yaw_from_quaternion(0, 0, np.sqrt(0.5), np.sqrt(0.5)) == pytest.approx(np.pi / 2)
    assert yaw_from_quaternion(0, 0, 0, 1) == 0
