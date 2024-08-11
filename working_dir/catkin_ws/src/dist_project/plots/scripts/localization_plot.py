import rosbag
import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":

    # Path of the rosbag file to extract data from
    # In order to take all the necessary data I edit this line when necessary
    bagfile = '/home/marco/shared/working_dir/catkin_ws/src/dist_project/bagfiles/few_tags.bag'
    # bagfile = '/home/marco/shared/working_dir/catkin_ws/src/dist_project/bagfiles/more_tags.bag'

    # I edit this lane when necessary to extract data from a particular topic
    topic1 = "/robot0/localization_data_topic"
    topic2 = "/robot0/ground_truth/state"

    # Empty lists to contain all data
    loc_data = []
    gt_data = []

    # Data extraction
    # Packing of data iterating all messages stored
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

    # Conversion of python list to numpy array
    loc_data = np.array(loc_data)
    gt_data = np.array(gt_data)

    # List of tag position
    tags = np.array(
        [[-10,-3],
        [-10,3],
        [-5,0],
        [4,3],
        [4,-3]]
    )
    
    # Data plotting
    plt.figure()
    plt.plot(loc_data[20:,0], loc_data[20:,1], label='Estimated position')
    plt.plot(gt_data[20:,0], gt_data[20:,1], label='Ground truth position')
    plt.plot(tags[:,0], tags[:,1], markersize=12, marker='*', linestyle='None', label='UWB tag')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Localization of single robot')
    plt.legend()
    plt.show()