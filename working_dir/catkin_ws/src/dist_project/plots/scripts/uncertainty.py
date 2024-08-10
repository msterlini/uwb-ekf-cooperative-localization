import rosbag
import matplotlib.pyplot as plt
import numpy as np


if __name__ == "__main__":

    # Path of the rosbag file to extract data from
    # In order to take all the necessary data I edit this line when necessary
    bagfile = '/home/marco/shared/working_dir/catkin_ws/src/dist_project/bagfiles/total_bag.bag'

    # Number of robots of the simulation
    n_robots = 5

    # Lists of total data
    gt = []
    loc = []
    target = []

    # Lists with topics names with respect to the robot
    gt_topics = []
    loc_topics = []
    target_topics = []

    # Topic list creation
    for i in range(n_robots):
        id = "robot" + str(i)
        gt_topics.append("/" + id + "/ground_truth/state")
        loc_topics.append("/" + id + "/localization_data_topic")
        target_topics.append("/" + id + "/target_estimate")

    # Ground truth data extraction
    with rosbag.Bag(bagfile, 'r') as bag:
        for i in range(n_robots):
            tmp = []
            for topic, msg, t in bag.read_messages(topics=gt_topics[i]):
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
                tmp.append([t.to_sec(), x, y])
            gt.append(np.array(tmp))

    gt = np.array(gt)

    # Localization data extraction
    with rosbag.Bag(bagfile, 'r') as bag:
        for i in range(n_robots):
            tmp = []
            for topic, msg, t in bag.read_messages(topics=loc_topics[i]):
                x = msg.position.x
                y = msg.position.y
                tmp.append([t.to_sec(), x, y])
            loc.append(np.array(tmp))

    loc = np.array(loc)

    # Target estimate data extraction
    with rosbag.Bag(bagfile, 'r') as bag:
        for i in range(n_robots):
            tmp = []
            for topic, msg, t in bag.read_messages(topics=target_topics[i]):
                x = msg.position.x
                y = msg.position.y
                z = msg.position.z
                tmp.append([t.to_sec(), x, y, z])
            target.append(np.array(tmp))
    
    target = np.array(target)

    # Data plotting
    plt.figure()
    for i in range(n_robots):
        plt.plot(loc[i][60:, 1], loc[i][60:, 2], label='Estimated position')
        plt.plot(gt[i][60:, 1], gt[i][60:, 2], label='Ground truth position')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('Multiple localization')
    plt.legend()
    plt.show()

    plt.figure()
    for i in range(n_robots):
        plt.plot(target[i][:, 0], target[i][:, 1], label='Estimated x')
        plt.plot(target[i][:, 0], target[i][:, 0]*0 + 1, markersize=0)
    plt.xlabel('X [m]')
    plt.ylabel('Time [s]')
    plt.title('Target x coordinate estimation')
    plt.legend()
    plt.show()

    plt.figure()
    for i in range(n_robots):
        plt.plot(target[i][:, 0], target[i][:, 2], label='Estimated y')
        plt.plot(target[i][:, 0], target[i][:, 0]*0 + 2, markersize=0)
    plt.xlabel('Y [m]')
    plt.ylabel('Time [s]')
    plt.title('Target y coordinate estimation')
    plt.legend()
    plt.show()

    plt.figure()
    for i in range(n_robots):
        plt.plot(target[i][:, 0], target[i][:, 3], label='Estimated z')
        plt.plot(target[i][:, 0], target[i][:, 0]*0 + 0.85, markersize=0)
    plt.xlabel('Z [m]')
    plt.ylabel('Time [s]')
    plt.title('Target z coordinate estimation')
    plt.legend()
    plt.show()

    for i in range(n_robots):
        start = len(gt[i]) - len(loc[i])
        delay = 70
        plt.figure()
        plt.hist(loc[i][delay:,1] - gt[i][start + delay:,1], bins=120)
        plt.xlabel('Error [m]')
        plt.ylabel('Frequency')
        plt.title('X localization error in robot ' + str(i))
        plt.show()

        plt.figure()
        plt.hist(loc[i][delay:,2] - gt[i][start + delay:,2], bins=120)
        plt.xlabel('Error [m]')
        plt.ylabel('Frequency')
        plt.title('Y localization error in robot ' + str(i))
        plt.show()

        plt.figure()
        plt.hist(target[i][:,1] - 1, bins=120)
        plt.xlabel('Error [m]')
        plt.ylabel('Frequency')
        plt.title('X localization error target of robot ' + str(i))
        plt.show()

        plt.figure()
        plt.hist(target[i][:,2] - 2, bins=120)
        plt.xlabel('Error [m]')
        plt.ylabel('Frequency')
        plt.title('Y localization error target of robot ' + str(i))
        plt.show()

        plt.figure()
        plt.hist(target[i][:,3] - 0.85, bins=120)
        plt.xlabel('Error [m]')
        plt.ylabel('Frequency')
        plt.title('Z localization error target of robot ' + str(i))
        plt.show()

        x_mean = np.mean(target[i][:,1])
        y_mean = np.mean(target[i][:,2])
        z_mean = np.mean(target[i][:,3])
        x_std = np.std(target[i][:,1])
        y_std = np.std(target[i][:,2])
        z_std = np.std(target[i][:,3])

        print('Robot ' + str(i) + ":")
        print("X coordinate: " + str(x_mean) + " +- " + str(x_std))
        print("Y coordinate: " + str(y_mean) + " +- " + str(y_std))
        print("Z coordinate: " + str(z_mean) + " +- " + str(z_std))
        print("")
        

    # print('X: mean, std and expected std')
    # print(np.mean(target_data[start:,1]))
    # print(np.std(target_data[start:,1]))
    # print(np.mean(target_data[start:,4]))
    # print()
    # print('Y: mean, std and expected std')
    # print(np.mean(target_data[start:,2]))
    # print(np.std(target_data[start:,2]))
    # print(np.mean(target_data[start:,5]))
    # print()
    # print('Z: mean, std and expected std')
    # print(np.mean(target_data[start:,3]))
    # print(np.std(target_data[start:,3]))
    # print(np.mean(target_data[start:,6]))
    # plt.figure()
    # # plt.hist(target_data[start:,1], bins=120)
    # plt.hist(target_data[start:,2], bins=120)
    # # plt.hist(target_data[start:,3] - np.mean(target_data[start:,3]), bins=120)
    # plt.xlabel('Error [m]')
    # plt.ylabel('Frequency')
    # plt.title('Error of x target estimation')
    # plt.legend()
    # plt.show()