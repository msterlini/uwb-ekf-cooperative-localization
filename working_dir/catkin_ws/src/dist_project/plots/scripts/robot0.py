import rosbag
import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
    bagfile = '/home/marco/shared/working_dir/catkin_ws/src/dist_project/bagfiles/robot0_bag.bag'

    topic1 = '/robot0/ground_truth/state'
    topic2 = '/robot0/localization_data_topic'
    topic3 = '/robot0/target_estimate'

    gt_data = []
    loc_data = []
    target_data = []

    with rosbag.Bag(bagfile, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[topic1]):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            gt_data.append([t.to_sec(), x, y])

    with rosbag.Bag(bagfile, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[topic2]):
            x = msg.position.x
            y = msg.position.y
            loc_data.append([t.to_sec(), x, y])

    with rosbag.Bag(bagfile, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[topic3]):
            x = msg.position.x
            y = msg.position.y
            z = msg.position.z
            sigma_x = msg.orientation.x
            sigma_y = msg.orientation.y
            sigma_z = msg.orientation.z
            target_data.append([t.to_sec(), x, y, z, sigma_x, sigma_y, sigma_z])

    gt_data = np.array(gt_data)
    loc_data = np.array(loc_data)
    target_data = np.array(target_data)

    # Data plotting
    plt.figure()
    plt.plot(loc_data[60:,1], loc_data[60:,2], label='Estimated position')
    plt.plot(gt_data[60:,1], gt_data[60:,2], label='Ground truth position')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('Localization of single robot')
    plt.legend()
    plt.show()

    delay = 69
    start = 70
    plt.figure()
    plt.hist(loc_data[start:,1] - gt_data[start + delay:, 1], bins=120)
    plt.xlabel('Error [m]')
    plt.ylabel('Frequency')
    plt.title('Error between ground truth and estimated data')
    plt.show()

    print('X: mean, std and expected std')
    print(np.mean(target_data[start:,1]))
    print(np.std(target_data[start:,1]))
    print(np.mean(target_data[start:,4]))
    print()
    print('Y: mean, std and expected std')
    print(np.mean(target_data[start:,2]))
    print(np.std(target_data[start:,2]))
    print(np.mean(target_data[start:,5]))
    print()
    print('Z: mean, std and expected std')
    print(np.mean(target_data[start:,3]))
    print(np.std(target_data[start:,3]))
    print(np.mean(target_data[start:,6]))
    plt.figure()
    # plt.hist(target_data[start:,1], bins=120)
    plt.hist(target_data[start:,2], bins=120)
    # plt.hist(target_data[start:,3] - np.mean(target_data[start:,3]), bins=120)
    plt.xlabel('Error [m]')
    plt.ylabel('Frequency')
    plt.title('Error of x target estimation')
    plt.legend()
    plt.show()