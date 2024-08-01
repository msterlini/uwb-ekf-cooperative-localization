import rosbag
import matplotlib.pyplot as plt
import numpy as np

# This function takes a bagfile path and a topic string and returns the time-labeled data array
def extract_data_topic(bagfile, topic):
    
    times = []
    data = []

    with rosbag.Bag(bagfile, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[topic]):
            times.append(t.to_sec())
            # I edit this line to extract the necessary data depending on the topic
            x = msg.position.x
            y = msg.position.y
            data.append([x, y])
    
    return np.array(times), np.array(data)

if __name__ == "__main__":

    # Path of the rosbag file to extract data from
    # In order to take all the necessary data I edit this line when necessary
    bagfile = '/home/marco/shared/working_dir/catkin_ws/src/dist_project/bagfiles/localization_robot0.bag'

    # I edit this lane when necessary to extract data from a particular topic
    topic1 = "/robot0/localization_data_topic"
    topic2 = "/robot0/ground_truth/state"

    loc_data = []
    gt_data = []

    with rosbag.Bag(bagfile, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[topic1]):
            x = msg.position.x
            y = msg.position.y
            loc_data.append([x, y])

    with rosbag.Bag(bagfile, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[topic2]):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            gt_data.append([x, y])

    loc_data = np.array(loc_data)
    gt_data = np.array(gt_data)
    
    # Data plotting
    plt.figure()
    plt.plot(loc_data[20:,0], loc_data[20:,1], label='Estimated position')
    plt.plot(gt_data[20:,0], gt_data[20:,1], label='Ground truth position')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Localization of single robot')
    plt.legend()
    plt.show()