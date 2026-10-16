import numpy as np


def yaw_from_quaternion(x, y, z, w):
    # Rotation around z of a quaternion, the robots move on the plane so roll and pitch are not needed
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def quaternion_from_yaw(yaw):
    # Quaternion (x, y, z, w) of a rotation of yaw around z
    return 0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2)
