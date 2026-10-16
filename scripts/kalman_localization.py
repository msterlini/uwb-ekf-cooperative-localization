#!/usr/bin/env python3
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time

from dist_project.anchors import get_anchors_pos
from dist_project.ekf import UnicycleEKF, wrap_angle
from dist_project.msg import UwbData
from dist_project.transforms import quaternion_from_yaw, yaw_from_quaternion


class KalmanEstimator(Node):

    def __init__(self):
        super().__init__('localization_kalman_filter', automatically_declare_parameters_from_overrides=True)
        param = lambda name: self.get_parameter(name).value  # noqa: E731

        # Filter initialization, mean vector of x, y, theta in zero and identity covariance matrix
        self.ekf = UnicycleEKF(
            v_std=(param("ekf_v_std"), param("ekf_v_std_rel")),
            w_std=(param("ekf_w_std"), param("ekf_w_std_rel")),
            range_rel_std=param("uwb_range_rel_std"),
            range_floor_std=param("uwb_range_floor_std"),
        )

        # Local variable to count the number of callback to wait before starting the motion planner
        self.actual_callback = 0
        self.number_of_callback = param("ekf_warmup_updates")
        self.heading_window = param("ekf_heading_window")
        self.heading_ready = False
        # Local variables to store the initial estimate to compare with a further one and estimate the orientation theta
        self.first_x = 0
        self.first_y = 0

        # Odometry and uwb callbacks could run on different threads with a multithreaded executor
        self.lock = threading.Lock()

        # Timestamp and pose of the last odometry message to compute the increments
        self.last_stamp = None
        self.last_odom = None
        self.default_dt = 1.0 / param("odom_rate")

        # Below this speed the robot is considered still and the uwb data don't update theta
        self.stationary_speed = param("ekf_stationary_speed")
        self.moving = True

        # Function to be called once, it scans for every uwb anchor in the area and stores their position
        self.tag_pos = get_anchors_pos(self)

        # Publisher for estimated state
        self.pub = self.create_publisher(PoseWithCovarianceStamped, 'localization_data_topic', 10)

        # Publisher to notify the motion planner to start, transient local so that it is received also if the
        # motion planner subscribes later
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.init_pub = self.create_publisher(Pose, 'init_move', latched)

        # Subscription to odom topic directly from gazebo simulation
        self.create_subscription(Odometry, 'odom', self.subscribe_odom_data, 10)

        # Subscription to uwb data topic directly from anchor simulation
        self.create_subscription(UwbData, 'uwb_data_topic', self.subscribe_uwb_data, 1)

    def prediction_step(self, odom_data: Odometry):
        # Delta t taken from the timestamps of the odometry messages
        stamp = Time.from_msg(odom_data.header.stamp).nanoseconds * 1e-9
        dt = self.default_dt if self.last_stamp is None else stamp - self.last_stamp
        if not 0 < dt < 1:
            dt = self.default_dt
        self.last_stamp = stamp

        # Velocity and rotation rate extraction from the increments of the odometry pose. The twist is an
        # instantaneous value, when the robot vibrates (e.g. stuck against a wall) integrating it makes theta drift
        pos = odom_data.pose.pose.position
        q = odom_data.pose.pose.orientation
        odom_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        if self.last_odom is None:
            delta_vel = odom_data.twist.twist.linear.x
            delta_w = odom_data.twist.twist.angular.z
        else:
            last_x, last_y, last_yaw = self.last_odom
            delta_vel = (np.cos(last_yaw) * (pos.x - last_x) + np.sin(last_yaw) * (pos.y - last_y)) / dt
            delta_w = wrap_angle(odom_yaw - last_yaw) / dt
        self.last_odom = (pos.x, pos.y, odom_yaw)
        self.moving = abs(delta_vel) > self.stationary_speed

        if self.heading_ready:
            self.ekf.predict(delta_vel, delta_w, dt)
        else:
            # Until theta is estimated I don't know in which direction the robot is moving,
            # so the movement only increases the uncertainty on x and y
            self.ekf.predict(0.0, delta_w, dt)
            self.ekf.sigma[:2, :2] += np.eye(2) * (delta_vel * dt)**2

        # Now it published the estimated state
        self.publish_data(odom_data.header.stamp)

    def correction_step(self, uwb_data: UwbData):
        # UWB data unpacking, each distance is associated to the anchor with the same id
        ids = np.array(uwb_data.tag_id, dtype=int)
        distances = np.array(uwb_data.distance)
        valid = ids < len(self.tag_pos)

        self.ekf.update(distances[valid], self.tag_pos[ids[valid]], update_heading=self.moving)

        # We wait for a user-designed number of callback to wait for the kalman filter to reach a good estimate
        # In this moment the robot is going in a straight line, we wait more callbacks to get a new position
        # and estimate theta as the result of a arctan2
        if not self.heading_ready:
            if self.actual_callback == self.number_of_callback:
                # We get the first coordinates to compare
                self.first_x, self.first_y = self.ekf.mu[0], self.ekf.mu[1]
            elif self.actual_callback == self.number_of_callback + self.heading_window:
                # I compute the initial corrected theta estimate
                dx = self.ekf.mu[0] - self.first_x
                dy = self.ekf.mu[1] - self.first_y
                self.ekf.mu[2] = wrap_angle(np.arctan2(dy, dx))

                # Uncertainty on theta given by the uncertainty on the position over the travelled distance
                pos_std = np.sqrt(self.ekf.sigma[0, 0] + self.ekf.sigma[1, 1])
                self.ekf.sigma[2, :] = 0
                self.ekf.sigma[:, 2] = 0
                self.ekf.sigma[2, 2] = (2 * pos_std / max(np.hypot(dx, dy), 1e-3))**2
                self.heading_ready = True

                # Send a message to tell the motion planner to start moving
                self.init_pub.publish(Pose())
            self.actual_callback += 1

    def subscribe_odom_data(self, odom_data: Odometry):
        # Callback function to the reception of a message from Odometry
        # With the data from the sensors on the unicyle I compute the prediction step
        with self.lock:
            self.prediction_step(odom_data)

    def subscribe_uwb_data(self, uwb_data: UwbData):
        # Callback function to the reception of a message from the uwb anchors
        # Now that I receive the data from other sensors I perform the correction step
        with self.lock:
            self.correction_step(uwb_data)

    def publish_data(self, stamp):

        # I declare the ROS msg PoseWithCovarianceStamped to publish
        robot_pos = PoseWithCovarianceStamped()
        robot_pos.header.stamp = stamp
        robot_pos.header.frame_id = 'world'

        x, y, theta = self.ekf.mu
        robot_pos.pose.pose.position.x = float(x)
        robot_pos.pose.pose.position.y = float(y)
        robot_pos.pose.pose.position.z = 0.0

        q = quaternion_from_yaw(theta)
        robot_pos.pose.pose.orientation.x = float(q[0])
        robot_pos.pose.pose.orientation.y = float(q[1])
        robot_pos.pose.pose.orientation.z = float(q[2])
        robot_pos.pose.pose.orientation.w = float(q[3])

        # The covariance is a 6x6 matrix on x, y, z, roll, pitch, yaw, I fill the elements of x, y and yaw
        covariance = np.zeros((6, 6))
        covariance[np.ix_([0, 1, 5], [0, 1, 5])] = self.ekf.sigma
        robot_pos.pose.covariance = covariance.flatten().tolist()

        # Finally I publish the state
        self.pub.publish(robot_pos)


if __name__ == "__main__":

    # Node initialization
    rclpy.init()

    # Class initialization
    estimator = KalmanEstimator()

    try:
        # We don't need to cast any particular function since the estimator already works on the callbacks of the topics
        rclpy.spin(estimator)
    except KeyboardInterrupt:
        estimator.get_logger().info("Program shutdown: Kalman filter estimator node interrupted")
