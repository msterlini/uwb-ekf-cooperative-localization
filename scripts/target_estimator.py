#!/usr/bin/env python3
import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from rclpy.node import Node

from dist_project.msg import RobotData
from dist_project.transforms import yaw_from_quaternion
from dist_project.triangulation import estimate_height, intersect_bearings


class TargetEstimator(Node):

    def __init__(self):
        super().__init__('target_estimator', automatically_declare_parameters_from_overrides=True)

        # Initialization of variables
        self.target_height = 0
        self.tag = RobotData()
        self.start_publishing = 0
        # The id of the robot is its namespace, e.g. robot0
        self.namespace = self.get_namespace().strip('/')

        # Target lectures from every robot, own included, indexed by robot id
        self.robot_tags = {}

        # Subscritpion to global topic to publish data to share
        self.pub_tag = self.create_publisher(RobotData, "/processing_data", 10)

        # Subscription to local topic to publish the final target estimate
        self.pub_target = self.create_publisher(PoseWithCovarianceStamped, "target_estimate", 1)

        # Subscritpion to local topic for target height
        self.sub_height = self.create_subscription(Point, "target_height", self.target_callback, 1)

        # Subscritpion to local topic for x,y values (robot position)
        self.sub_pos = self.create_subscription(PoseWithCovarianceStamped, "localization_data_topic",
                                                self.localization_callback, 1)

        # Start self-tag publishing at a fixed rate
        self.create_timer(1.0 / self.get_parameter("target_estimator_rate").value, self.publish_tag)

    def publish_tag(self):
        if self.start_publishing:
            self.pub_tag.publish(self.tag)

    def target_callback(self, data):
        # Everytime the robots receives a new data from the local camera it uploads the current stored value for the
        # target height
        self.target_height = -data.z

    def localization_callback(self, actual_pos):
        # If it has already stored a value for the target height it means the robot is on position and then can start
        # publishing the data to the others robot
        if self.target_height == 0:
            return

        pose = actual_pos.pose.pose
        q = pose.orientation
        theta = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        covariance = np.reshape(actual_pos.pose.covariance, (6, 6))

        # Compilation of the self-tag with all the informations, the uncertainties are standard deviations
        tag = RobotData()
        tag.robot_id = self.namespace
        tag.target_height = self.target_height
        tag.x = pose.position.x
        tag.y = pose.position.y
        tag.theta = float(theta)
        tag.sigma_x = np.sqrt(covariance[0, 0])
        tag.sigma_y = np.sqrt(covariance[1, 1])
        tag.sigma_theta = np.sqrt(covariance[5, 5])
        self.tag = tag

        # Local variable to tell the module to start publishing
        if not self.start_publishing:
            # Subscription to global topic for communication
            self.sub_data = self.create_subscription(RobotData, "/processing_data", self.gather_callback, 50)
            self.start_publishing = 1

    def gather_callback(self, data):
        # Called upon reception of tags from other robots, the tag of the same robot gets updated
        self.robot_tags[data.robot_id] = data

        # When the list contains at least 2 different tags from 2 robots it estimates the target position
        if len(self.robot_tags) < 2:
            return

        tags = list(self.robot_tags.values())
        robots = [[t.x, t.y, t.theta, t.sigma_x, t.sigma_y, t.sigma_theta] for t in tags]

        # Target position with uncertainties extraction
        p, cov_matrix = intersect_bearings(robots)
        if p is None:
            return

        # Once the plane position of the target has been computed its height estimation will be carried out
        target_z, sigma_target_z = estimate_height(p, robots, [t.target_height for t in tags])

        # Composition and publishing of target estimate
        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'world'
        message.pose.pose.position.x = float(p[0])
        message.pose.pose.position.y = float(p[1])
        message.pose.pose.position.z = float(target_z)
        message.pose.pose.orientation.w = 1.0

        covariance = np.zeros((6, 6))
        covariance[:2, :2] = cov_matrix
        covariance[2, 2] = sigma_target_z**2
        message.pose.covariance = covariance.flatten().tolist()

        self.pub_target.publish(message)


if __name__ == '__main__':

    # Node initialization
    rclpy.init()

    # Class initialization
    target_estimator = TargetEstimator()

    try:
        rclpy.spin(target_estimator)
    except KeyboardInterrupt:
        target_estimator.get_logger().info("Program shutdown: Target estimator node interrupted")
