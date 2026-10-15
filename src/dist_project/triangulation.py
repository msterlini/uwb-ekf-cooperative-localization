import numpy as np

# Vertical tilt of the camera taken out empirically
VERTICAL_TILT = 67 * np.pi / 180


def intersect_bearings(robots, iterations=3):
    # Each row of robots is [x, y, theta, sigma_x, sigma_y, sigma_theta] of a robot pointing at the target.
    # Each robot defines a line, I write it as -sin(theta)*x + cos(theta)*y = c so that vertical lines are not a problem
    robots = np.atleast_2d(np.asarray(robots, dtype=float))

    # At least 2 lines are needed to find an intersection
    if robots.shape[0] < 2:
        return None, None

    x, y, theta = robots[:, 0], robots[:, 1], robots[:, 2]
    sigma_x, sigma_y, sigma_theta = robots[:, 3], robots[:, 4], robots[:, 5]

    # Matrix initialization
    A = np.column_stack((-np.sin(theta), np.cos(theta)))
    b = A[:, 0] * x + A[:, 1] * y

    # Parallel lines don't have an intersection
    if np.linalg.cond(A) > 1e6:
        return None, None

    # The uncertainty of each line depends on the distance from the target, so I start with equal weights
    # and I recompute them a few times with the last estimate
    weights = np.ones(len(robots))
    for _ in range(iterations + 1):
        C_inv = np.diag(weights)

        # Solving the weighted LSQ problem p = (A^T.C^-1.A)^-1 . A^T.C^-1.b
        cov_matrix = np.linalg.inv(A.T @ C_inv @ A)
        p = cov_matrix @ A.T @ C_inv @ b

        # Law of propagation of uncertainties on the distance of the target from each line
        distance = np.hypot(p[0] - x, p[1] - y)
        variance = (np.sin(theta) * sigma_x)**2 + (np.cos(theta) * sigma_y)**2 + (distance * sigma_theta)**2
        weights = 1.0 / np.maximum(variance, 1e-6)

    # Returns target position with its covariance matrix
    return p, cov_matrix


def estimate_height(target_xy, robots, target_heights, sigma_vertical_angle=VERTICAL_TILT * 0.001):
    # z-coordinate estimation considering the vertical fov is 67 degrees.
    # At a value of target_height of 1 corresponds 67 degrees. For 0 is 0 degrees.
    robots = np.atleast_2d(np.asarray(robots, dtype=float))
    target_x, target_y = target_xy

    # Plane distance from robot and target
    dx = robots[:, 0] - target_x
    dy = robots[:, 1] - target_y
    distance = np.maximum(np.hypot(dx, dy), 1e-6)

    # Vertical angle computation proportional to target_height value
    vertical_angle = np.asarray(target_heights, dtype=float) * VERTICAL_TILT

    # Direct consequence from the definition of tangent
    quotes = np.tan(vertical_angle) * distance

    # Uncertainty computation via law of propagation of error
    sigma_distance = np.sqrt((dx / distance * robots[:, 3])**2 + (dy / distance * robots[:, 4])**2)
    sigma_quotes = np.sqrt((np.tan(vertical_angle) * sigma_distance)**2
                           + (distance / np.cos(vertical_angle)**2 * sigma_vertical_angle)**2)

    # Weighted mean of all the computed values
    weights = 1 / sigma_quotes**2
    target_z = np.sum(weights * quotes) / np.sum(weights)
    sigma_target_z = np.sqrt(1 / np.sum(weights))

    return target_z, sigma_target_z
