import numpy as np


def wrap_angle(angle):
    # I modify the angle in order to obtain a value in [-pi, pi]
    return (angle + np.pi) % (2 * np.pi) - np.pi


def range_std(distance, rel_std, floor_std):
    # The noise of a UWB measurement is proportional to the actual distance, plus a constant term
    return floor_std + rel_std * np.asarray(distance, dtype=float)


class UnicycleEKF:

    def __init__(self, mu0=None, sigma0=None, v_std=(0.05, 0.1), w_std=(0.05, 0.1),
                 range_rel_std=0.015, range_floor_std=0.05):

        # Mean vector of x, y, theta initialization
        self.mu = np.zeros(3) if mu0 is None else np.array(mu0, dtype=float)

        # Covariance matrix initialization
        self.sigma = np.eye(3) if sigma0 is None else np.array(sigma0, dtype=float)

        # Noise on the odometry inputs: a constant value plus a contribute proportional to the actual value read
        self.v_std = v_std
        self.w_std = w_std

        # Noise on the UWB measurements
        self.range_rel_std = range_rel_std
        self.range_floor_std = range_floor_std

    def predict(self, v, w, dt):
        # Local variable initialization
        x, y, theta = self.mu

        # Update of the state with model knowledge
        x_new = x + v * np.cos(theta) * dt
        y_new = y + v * np.sin(theta) * dt
        theta_new = wrap_angle(theta + w * dt)

        # Jacobian of the dynamics with respect to the state
        A = np.array([[1.0, 0.0, -v * np.sin(theta) * dt],
                      [0.0, 1.0, v * np.cos(theta) * dt],
                      [0.0, 0.0, 1.0]])

        # Kinematic model noise redistribution
        G = np.array([[np.cos(theta) * dt, 0.0],
                      [np.sin(theta) * dt, 0.0],
                      [0.0, dt]])

        # Covariance matrix of the process
        sv = self.v_std[0] + self.v_std[1] * abs(v)
        sw = self.w_std[0] + self.w_std[1] * abs(w)
        Q = np.array([[sv**2, 0.0],
                      [0.0, sw**2]])

        # New mean vector and covariance matrix
        self.mu = np.array([x_new, y_new, theta_new])
        self.sigma = (A @ self.sigma) @ A.T + (G @ Q) @ G.T

    def expected_ranges(self, anchors):
        # Distance of each anchor from the current estimate, I put 0 for z since the robot is bounded to the ground
        robot = np.array([self.mu[0], self.mu[1], 0.0])
        return np.linalg.norm(np.atleast_2d(anchors) - robot, axis=1)

    def update(self, ranges, anchors, update_heading=True):
        Z = np.asarray(ranges, dtype=float)
        anchors = np.atleast_2d(np.asarray(anchors, dtype=float))
        if Z.size == 0:
            return Z

        # Local variable initialization
        x, y, theta = self.mu

        # For expected distances I intend the result of the model of the measuring system
        expected_distances = self.expected_ranges(anchors)

        # I build up the H matrix, the measurements don't depend on theta
        H = np.zeros((Z.size, 3))
        H[:, 0] = (x - anchors[:, 0]) / expected_distances
        H[:, 1] = (y - anchors[:, 1]) / expected_distances

        # Covariance matrix of measurement process, diagonal since every anchor is independent
        R = np.diag(range_std(expected_distances, self.range_rel_std, self.range_floor_std)**2)

        # I define the matrix S_inv as the inverse of matrix S = H.sigma.H^T + R
        S_inv = np.linalg.inv((H @ self.sigma) @ H.T + R)

        # I define the Kalman gain as the matrix W
        W = (self.sigma @ H.T) @ S_inv

        # When the robot is still the distances can't tell anything about theta, without this the noise of the
        # measurements would reach theta through the correlation with x and y and make it drift
        if not update_heading:
            W[2, :] = 0

        # Mean vector update
        innovation = Z - expected_distances
        self.mu = self.mu + W @ innovation
        self.mu[2] = wrap_angle(self.mu[2])

        # Sigma update, written in Joseph form to keep it symmetric
        I_WH = np.eye(3) - W @ H
        self.sigma = I_WH @ self.sigma @ I_WH.T + W @ R @ W.T

        return innovation

    @property
    def std(self):
        return np.sqrt(np.diag(self.sigma))
