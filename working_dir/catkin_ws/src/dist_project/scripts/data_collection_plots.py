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
    bagfile = '/home/marco/shared/working_dir/catkin_ws/src/dist_project/bagfiles/robot0_loc.bag'

    # I edit this lane when necessary to extract data from a particular topic
    topic = "/robot0/localization_data_topic"

    times, data = extract_data_topic(bagfile, topic)

    # Data plotting
    plt.figure()
    plt.plot(data[20:,0], data[20:,1])
    plt.show()

    pass