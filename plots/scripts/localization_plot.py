import os
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from rosbags.highlevel import AnyReader

if __name__ == "__main__":

    # Path of the rosbag file to extract data from, few_tags.bag by default
    bagfiles_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'bagfiles')
    bagfile = sys.argv[1] if len(sys.argv) > 1 else os.path.join(bagfiles_dir, 'few_tags.bag')

    # I edit this lane when necessary to extract data from a particular topic
    topic1 = "/robot0/localization_data_topic"
    topic2 = "/robot0/ground_truth/state"

    # Empty lists to contain all data
    loc_data = []
    gt_data = []

    # Data extraction
    # Packing of data iterating all messages stored
    with AnyReader([Path(bagfile)]) as reader:
        for connection, t, rawdata in reader.messages():
            msg = reader.deserialize(rawdata, connection.msgtype)
            if connection.topic == topic1:
                # The old bag files store a Pose, the new ones a PoseWithCovarianceStamped
                pose = msg.pose.pose if hasattr(msg, 'pose') else msg
                loc_data.append([pose.position.x, pose.position.y])
            elif connection.topic == topic2:
                gt_data.append([msg.pose.pose.position.x, msg.pose.pose.position.y])

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
