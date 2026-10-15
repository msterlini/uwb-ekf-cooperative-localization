#!/usr/bin/env python3
import numpy as np
import rospy
import tf.transformations
from geometry_msgs.msg import Point, PoseWithCovarianceStamped

from dist_project.msg import robot_data
from dist_project.triangulation import estimate_height, intersect_bearings


class TargetEstimator:

    def __init__(self, namespace):

        # Initialization of variables
        self.target_height = 0
        self.tag = robot_data()
        self.start_publishing = 0
        self.namespace = namespace

        # Target lectures from every robot, own included, indexed by robot id
        self.robot_tags = {}

        # Subscritpion to global topic to publish data to share
        self.pub_tag = rospy.Publisher("/processing_data", robot_data, queue_size=10)

        # Subscription to local topic to publish the final target estimate
        self.pub_target = rospy.Publisher("target_estimate", PoseWithCovarianceStamped, queue_size=1)

        # Subscritpion to local topic for target height
        self.sub_height = rospy.Subscriber("target_height", Point, self.target_callback, queue_size=1)

        # Subscritpion to local topic for x,y values (robot position)
        self.sub_pos = rospy.Subscriber("localization_data_topic", PoseWithCovarianceStamped,
                                        self.localization_callback, queue_size=1)

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
        (roll, pitch, theta) = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        covariance = np.reshape(actual_pos.pose.covariance, (6, 6))

        # Compilation of the self-tag with all the informations, the uncertainties are standard deviations
        tag = robot_data()
        tag.robot_id = self.namespace
        tag.target_height = self.target_height
        tag.x = pose.position.x
        tag.y = pose.position.y
        tag.theta = theta
        tag.sigma_x = np.sqrt(covariance[0, 0])
        tag.sigma_y = np.sqrt(covariance[1, 1])
        tag.sigma_theta = np.sqrt(covariance[5, 5])
        self.tag = tag

        # Local variable to tell the module to start publishing
        if not self.start_publishing:
            # Subscription to global topic for communication
            self.sub_data = rospy.Subscriber("/processing_data", robot_data, self.gather_callback, queue_size=50)
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
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = 'world'
        message.pose.pose.position.x = p[0]
        message.pose.pose.position.y = p[1]
        message.pose.pose.position.z = target_z
        message.pose.pose.orientation.w = 1

        covariance = np.zeros((6, 6))
        covariance[:2, :2] = cov_matrix
        covariance[2, 2] = sigma_target_z**2
        message.pose.covariance = covariance.flatten().tolist()

        self.pub_target.publish(message)


if __name__ == '__main__':

    # Node initialization
    rospy.init_node('target_estimator', anonymous=True)

    # Parameters import
    rate_val = rospy.get_param("/target_estimator_rate")
    id = rospy.get_param('~namespace')
    namespace = "robot" + str(id)

    # Class initialization
    target_estimator = TargetEstimator(namespace)

    # Rospy rate of node imposition
    rate = rospy.Rate(rate_val)

    try:
        while not rospy.is_shutdown():
            # Start self-tag publishing
            if target_estimator.start_publishing:
                target_estimator.pub_tag.publish(target_estimator.tag)
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program shutdown: Target estimator node interrupted")
