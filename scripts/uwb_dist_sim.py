#!/usr/bin/env python3
import numpy as np
import rospy
from nav_msgs.msg import Odometry

from dist_project.anchors import get_anchors_pos
from dist_project.ekf import range_std
from dist_project.msg import uwb_data


# Class creation
class UwbSimulation:

    def __init__(self):
        # Node initialization
        rospy.init_node('uwb_simulation', anonymous=True)

        # Topic to publish the values of the distances of the robot from the anchors
        self.pub = rospy.Publisher('uwb_data_topic', uwb_data, queue_size=10)

        # Noise parameters, the same used by the kalman filter
        self.rel_std = rospy.get_param("/uwb_range_rel_std")
        self.floor_std = rospy.get_param("/uwb_range_floor_std")

        # Function to be called once, it scans for every uwb anchor in the area and stores their position
        self.anchor_pos = get_anchors_pos()

        # Initialization of current ground truth pose of the robot
        self.robot_pose = None

        # Subscription to local ground_truth topic coming from the robot, I only need the last position
        rospy.Subscriber('ground_truth/state', Odometry, self.subscribe_data, queue_size=1)

        # The distances are computed and published at a fixed rate
        rospy.Timer(rospy.Duration(1.0 / rospy.get_param("/uwb_rate")), self.uwb_sim)

    def subscribe_data(self, data):
        # I update the actual position of the robot
        self.robot_pose = data.pose.pose.position

    def uwb_sim(self, event):
        if self.robot_pose is None or len(self.anchor_pos) == 0:
            return

        # For each anchor I calculate its distance from the robot along with its id
        ids = list(range(len(self.anchor_pos)))
        distances = [self.calculate_distance(pos) for pos in self.anchor_pos]

        # Once the distances have been calculated these informations can be published
        self.publish_data(ids, distances)

    def calculate_distance(self, uwb_pose):
        # To compute the distance I simplify the process in the simulation by adding noise to the distance taken from the
        # ground truth topic
        p1 = np.array(uwb_pose)
        p2 = np.array([self.robot_pose.x, self.robot_pose.y, 0])
        uwb_dist = np.linalg.norm(p1 - p2)

        # The noise I add is proportional to the actual measured distance, the closer the robot is to a tag the more
        # precise the measurement will be
        return float(uwb_dist + np.random.normal(0, range_std(uwb_dist, self.rel_std, self.floor_std)))

    def publish_data(self, ids, distances):
        # I initialize the message defined in the msg folder
        uwb_data_cell = uwb_data()
        uwb_data_cell.tag_id = ids
        uwb_data_cell.distance = distances
        # I publish the uwb data
        self.pub.publish(uwb_data_cell)


if __name__ == "__main__":

    # Initialize simulator class
    sim = UwbSimulation()

    # Start and run node
    rospy.spin()
