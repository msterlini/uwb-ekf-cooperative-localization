import tf
import rospy
from dist_project.msg import uwb_data
import time
import numpy as np
from nav_msgs.msg import Odometry
import sys

# Parameters import
init_time = rospy.get_param("/initialization_time")

# Node initialization
rospy.init_node('uwb_simulation', anonymous=True)
# Topic to publish the values of the distances of the robot from the anchors
pub = rospy.Publisher('uwb_data_topic', uwb_data, queue_size=10)


# Function to be called once, it scans for every uwb anchor in the area and stores their position
def get_anchors_pos():
    # Maximum number of anchors allowable 
    max_anchors = 20
    sensor_pos = []
    # This is relative to how I declared the id of each anchor in the file spawn_tags.py. I decided to name them uwb_anchor_0 with increasing numbers
    uwb_id = 'uwb_anchor_'
    # I start the listener from the package tf to get the transformation between the frame world and each tag. This was a easy way to implement it dynamically since we retrieve the informations of the anchors directly by the simulation. 
    listener = tf.TransformListener()

    for i in range(max_anchors):
        # For each possible tag name
        try:
            time.sleep(0.5)
            # I store the translation of each tag w.r.t. world frame
            (trans, rot) = listener.lookupTransform('/world', uwb_id + str(i), rospy.Time(0))
            # I store the position found
            sensor_pos.append(trans)
        # Error handling
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, KeyboardInterrupt):
            break

    if sensor_pos == []:
        # In case no uwb anchor was detected I run the function again. Usually this won't happen since I tried to make this function run after some initialization time interval
        rospy.logwarn("No UWb anchor detected. Trying again")
        get_anchors_pos()
    else:
        # Just for debugging
        rospy.loginfo("UWB Anchor list:\n" + str(sensor_pos))

    return sensor_pos

def calculate_distance(uwb_pose):
    # To compute the distance I simplify the process in the simulation by adding noise the the distance taken from the topic /ground_truth
    p1 = np.array(uwb_pose)
    p2 = np.array([robot_pose.x, robot_pose.y, 0])

    uwb_dist = np.sum((p1 - p2)**2, axis=0)
    # The noise I add is proportional to the actual measured distance, the closer the robot is to the tag, the more precise will be the measurement
    uwb_dist = uwb_dist + np.random.normal(0, uwb_dist*0.015, 1)
    return np.sqrt(uwb_dist)

def uwb_simulate():
    time.sleep(0.1)
    all_distance = []
    all_destination_id = []

    for i in range(len(sensor_pos)):
        dist = calculate_distance(sensor_pos[i])
        all_distance.append(dist)
        all_destination_id.append(i)
    
    # I publish data regarding the id of each anchor and the correspective measured distance

    publish_data(all_destination_id, all_distance)

def publish_data(all_destination_id, all_distance):
    # I initialize the message defined in msg folder
    uwb_data_cell = uwb_data()
    uwb_data_cell.destination_id = all_destination_id
    uwb_data_cell.distance = all_distance
    print(uwb_data_cell)
    pub.publish(uwb_data_cell)

# Callback function to be called whenever I receive data from ground_truth topic
def subscribe_data(data):
    global robot_pose
    robot_pose = data.pose.pose.position
    uwb_simulate()
        

if __name__ == "__main__":
    # Initialization time interval
    time.sleep(init_time)

    global sensor_pos
    sensor_pos = get_anchors_pos()
    print("Number of tag found: " + str(len(sensor_pos)))

    # Subscription to ground_truth topic
    rospy.Subscriber('ground_truth/state', Odometry, subscribe_data)

    rospy.spin()
