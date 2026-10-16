#!/usr/bin/env python3
import csv
import threading

import matplotlib
import numpy as np
import rospy
import tf.transformations
from geometry_msgs.msg import PoseWithCovarianceStamped
from matplotlib.patches import Ellipse
from nav_msgs.msg import Odometry

matplotlib.use("Qt5Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.animation import FuncAnimation  # noqa: E402

# The covariance ellipses of the robots are of a few centimeters, I scale them to make them visible
ELLIPSE_SCALE = 10
# Height of the center of the yellow sphere once the target lies on the floor
TARGET_Z = 0.8
# Estimates are shown only once the filter converged, before the robot doesn't know where it is
CONVERGED_STD = 0.2
COLORS = ["tab:orange", "tab:green", "tab:pink", "tab:blue", "tab:purple", "tab:brown", "tab:olive", "tab:cyan"]


def read_coordinates(path):
    with open(path) as f:
        return np.array([[float(x) for x in row] for row in csv.reader(f) if row])


def yaw(q):
    return tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def covariance_ellipse(mean, cov, scale, **kwargs):
    # 2-sigma ellipse from the eigen-decomposition of the covariance matrix
    values, vectors = np.linalg.eigh(cov)
    values = np.maximum(values, 0)
    angle = np.degrees(np.arctan2(vectors[1, 1], vectors[0, 1]))
    width, height = 2 * 2 * scale * np.sqrt(values[::-1])
    return Ellipse(mean, width, height, angle=angle, fill=False, **kwargs)


class LiveMonitor:

    def __init__(self):
        # Scene read from the same csv files used to spawn it
        self.anchors = read_coordinates(rospy.get_param("/tag_coordinate_file"))
        self.target = np.append(read_coordinates(rospy.get_param("/target_coordinate_file"))[0], TARGET_Z)
        self.n_robots = len(read_coordinates(rospy.get_param("/robot_coordinate_file")))

        # Callbacks run in other threads than the plotting
        self.lock = threading.Lock()

        # For each robot the history of ground truth and estimate, the last covariance and the position errors
        self.gt = [[] for _ in range(self.n_robots)]
        self.est = [[] for _ in range(self.n_robots)]
        self.cov = [None] * self.n_robots
        self.errors = [[] for _ in range(self.n_robots)]
        self.last_gt = {}
        # Last cooperative target estimate with its covariance and the history of its error
        self.target_estimate = None
        self.target_errors = []
        self.t0 = None

        for i in range(self.n_robots):
            ns = "/robot" + str(i)
            rospy.Subscriber(ns + "/ground_truth/state", Odometry, self.gt_callback, i, queue_size=1)
            rospy.Subscriber(ns + "/localization_data_topic", PoseWithCovarianceStamped, self.loc_callback, i,
                             queue_size=1)
            rospy.Subscriber(ns + "/target_estimate", PoseWithCovarianceStamped, self.target_callback,
                             queue_size=1)

    def time(self):
        t = rospy.get_time()
        if self.t0 is None:
            self.t0 = t
        return t - self.t0

    def gt_callback(self, msg, i):
        p = msg.pose.pose.position
        with self.lock:
            # The ground truth is published at a high rate, I keep a point every 5 cm
            if not self.gt[i] or np.hypot(p.x - self.gt[i][-1][0], p.y - self.gt[i][-1][1]) > 0.05:
                self.gt[i].append((p.x, p.y))
            self.last_gt[i] = (p.x, p.y)

    def loc_callback(self, msg, i):
        pose = msg.pose.pose
        cov = np.reshape(msg.pose.covariance, (6, 6))[:2, :2]
        if np.sqrt(np.trace(cov)) > CONVERGED_STD:
            return
        with self.lock:
            self.est[i].append((pose.position.x, pose.position.y, yaw(pose.orientation)))
            self.cov[i] = cov
            last_gt = self.last_gt.get(i)
            if last_gt is not None:
                error = np.hypot(pose.position.x - last_gt[0], pose.position.y - last_gt[1])
                self.errors[i].append((self.time(), error))

    def target_callback(self, msg):
        p = msg.pose.pose.position
        cov = np.reshape(msg.pose.covariance, (6, 6))
        with self.lock:
            self.target_estimate = (np.array([p.x, p.y, p.z]), cov[:2, :2], np.sqrt(cov[2, 2]))
            error = np.linalg.norm(self.target_estimate[0] - self.target)
            if not self.target_errors or self.time() - self.target_errors[-1][0] > 0.2:
                self.target_errors.append((self.time(), error))

    def setup(self):
        self.fig = plt.figure("dist_project - live monitor", figsize=(13, 6.5))
        grid = self.fig.add_gridspec(2, 2, width_ratios=[1.5, 1])
        self.ax_map = self.fig.add_subplot(grid[:, 0])
        self.ax_err = self.fig.add_subplot(grid[0, 1])
        self.ax_target = self.fig.add_subplot(grid[1, 1])

        # Map limits from the room walls
        xlim = rospy.get_param("/monitor_xlim", [-7.2, 7.8])
        ylim = rospy.get_param("/monitor_ylim", [-4.3, 5.6])
        self.ax_map.set_xlim(xlim)
        self.ax_map.set_ylim(ylim)
        self.ax_map.set_aspect("equal")
        self.ax_map.set_xlabel("x [m]")
        self.ax_map.set_ylabel("y [m]")
        self.ax_map.set_title("Robots (dashed: ground truth) and target")
        self.ax_map.plot(self.anchors[:, 0], self.anchors[:, 1], "s", color="tab:blue", label="UWB anchors")
        self.ax_map.plot(*self.target[:2], "o", markersize=12, markerfacecolor="none", color="gold",
                         markeredgewidth=2, label="Target (true)")

        self.gt_lines, self.est_lines, self.heads, self.ellipses = [], [], [], []
        for i in range(self.n_robots):
            c = COLORS[i % len(COLORS)]
            self.gt_lines.append(self.ax_map.plot([], [], "--", color=c, alpha=0.5, linewidth=1)[0])
            self.est_lines.append(self.ax_map.plot([], [], "-", color=c, linewidth=1.5, label="robot" + str(i))[0])
            self.heads.append(self.ax_map.plot([], [], marker=(3, 0, 0), markersize=11, color=c)[0])
            self.ellipses.append(None)
        self.target_marker = self.ax_map.plot([], [], "x", color="black", markersize=12, markeredgewidth=2,
                                              label="Target estimate")[0]
        self.target_ellipse = None
        self.target_text = self.ax_map.text(0.02, 0.02, "", transform=self.ax_map.transAxes, fontsize=10,
                                            bbox=dict(facecolor="white", alpha=0.8, edgecolor="none"))
        self.ax_map.text(0.98, 0.02, "robot ellipses x%d" % ELLIPSE_SCALE, transform=self.ax_map.transAxes,
                         fontsize=8, ha="right", color="gray")
        self.ax_map.legend(loc="upper right", fontsize=8, ncol=2)

        self.err_lines = [self.ax_err.plot([], [], color=COLORS[i % len(COLORS)])[0] for i in range(self.n_robots)]
        self.ax_err.set_title("Localization error")
        self.ax_err.set_ylabel("[m]")
        self.ax_err.grid(alpha=0.3)

        self.target_line = self.ax_target.plot([], [], color="black")[0]
        self.ax_target.set_title("Target estimate error")
        self.ax_target.set_xlabel("time [s]")
        self.ax_target.set_ylabel("[m]")
        self.ax_target.grid(alpha=0.3)
        self.fig.tight_layout()

    def update(self, frame):
        with self.lock:
            gt = [list(g) for g in self.gt]
            est = [list(e) for e in self.est]
            cov = list(self.cov)
            errors = [list(e) for e in self.errors]
            target_estimate = self.target_estimate
            target_errors = list(self.target_errors)

        for i in range(self.n_robots):
            if gt[i]:
                g = np.array(gt[i])
                self.gt_lines[i].set_data(g[:, 0], g[:, 1])
            if est[i]:
                e = np.array(est[i])
                self.est_lines[i].set_data(e[:, 0], e[:, 1])
                x, y, theta = e[-1]
                self.heads[i].set_data([x], [y])
                self.heads[i].set_marker((3, 0, np.degrees(theta) - 90))
                if self.ellipses[i] is not None:
                    self.ellipses[i].remove()
                self.ellipses[i] = covariance_ellipse((x, y), cov[i], ELLIPSE_SCALE, color=self.est_lines[i].get_color(),
                                                      linewidth=1)
                self.ax_map.add_patch(self.ellipses[i])
            if errors[i]:
                err = np.array(errors[i])
                self.err_lines[i].set_data(err[:, 0], err[:, 1])

        if target_estimate is not None:
            p, cov_xy, sigma_z = target_estimate
            self.target_marker.set_data([p[0]], [p[1]])
            if self.target_ellipse is not None:
                self.target_ellipse.remove()
            self.target_ellipse = covariance_ellipse(p[:2], cov_xy, 1, color="black", linewidth=1, linestyle=":")
            self.ax_map.add_patch(self.target_ellipse)
            self.target_text.set_text("target estimate  x %.2f  y %.2f  z %.2f m\nerror %.3f m"
                                      % (p[0], p[1], p[2], np.linalg.norm(p - self.target)))
        if target_errors:
            te = np.array(target_errors)
            self.target_line.set_data(te[:, 0], te[:, 1])

        for ax in (self.ax_err, self.ax_target):
            ax.relim()
            ax.autoscale_view()


if __name__ == "__main__":

    # Node initialization
    rospy.init_node("live_monitor", anonymous=True)

    # Class initialization
    monitor = LiveMonitor()
    monitor.setup()

    # The figure is refreshed a few times per second, the data arrive in the callbacks
    animation = FuncAnimation(monitor.fig, monitor.update, interval=300)
    plt.show()
