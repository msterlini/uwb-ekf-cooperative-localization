import rosbag
import matplotlib.pyplot as plt
import numpy as np
import tf.transformations


if __name__ == "__main__":

    # Path of the rosbag file to extract data from
    # In order to take all the necessary data I edit this line when necessary
    bagfile = '/home/marco/shared/working_dir/catkin_ws/src/dist_project/bagfiles/total_bag.bag'

    # Number of robots of the simulation
    n_robots = 5

    # Lists of total data
    gt = []
    loc = []
    loc_std = []
    target = []
    target_std = []

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
                explicit_quat = [
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w
                ]
                (roll, pitch, theta) = tf.transformations.euler_from_quaternion(explicit_quat)
                tmp.append([t.to_sec(), x, y, theta])
            gt.append(np.array(tmp))

    gt = np.array(gt)

    # Localization data extraction
    with rosbag.Bag(bagfile, 'r') as bag:
        for i in range(n_robots):
            tmp1 = []
            tmp2 = []
            for topic, msg, t in bag.read_messages(topics=loc_topics[i]):
                x = msg.position.x
                y = msg.position.y
                theta = msg.orientation.z
                sigma_x = msg.orientation.x
                sigma_y = msg.orientation.y
                sigma_theta = msg.orientation.w
                tmp1.append([t.to_sec(), x, y, theta])
                tmp2.append([t.to_sec(), sigma_x, sigma_y, sigma_theta])
            loc.append(np.array(tmp1))
            loc_std.append(np.array(tmp2))

    loc = np.array(loc)
    loc_std = np.array(loc_std)

    # Target estimate data extraction
    with rosbag.Bag(bagfile, 'r') as bag:
        for i in range(n_robots):
            tmp1 = []
            tmp2 = []
            for topic, msg, t in bag.read_messages(topics=target_topics[i]):
                x = msg.position.x
                y = msg.position.y
                z = msg.position.z
                sigma_x = msg.orientation.x
                sigma_y = msg.orientation.y
                sigma_z = msg.orientation.z
                tmp1.append([t.to_sec(), x, y, z])
                tmp2.append([t.to_sec(), sigma_x, sigma_y, sigma_z])
            target.append(np.array(tmp1))
            target_std.append(np.array(tmp2))

    
    target = np.array(target)
    target_std = np.array(target_std)

    # List of tag position
    tags = np.array([
        [-3,-3],
        [-3,3],
        [0,-3],
        [0,3],
        [3,-3],
        [3,3]]
    )

    # Data plotting
    plt.figure()
    for i in range(n_robots):
        plt.plot(loc[i][60:, 1], loc[i][60:, 2], label='Estimate robot ' + str(i))
        plt.plot(gt[i][60:, 1], gt[i][60:, 2], label='G-T robot ' + str(i))
    plt.plot(tags[:,0], tags[:,1], label='UWB tags', markersize=12, linestyle='None', marker='*', color='blue') 
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('Multiple localization')
    plt.legend()
    plt.show()

    plt.figure()
    for i in range(n_robots):
        plt.plot(target[i][:, 0], target[i][:, 1], label='Estimated x robot ' + str(i))
        plt.plot(target[i][:, 0], target[i][:, 0]*0 + 1, markersize=0)
    plt.xlabel('X [m]')
    plt.ylabel('Time [s]')
    plt.title('Target x coordinate estimation')
    plt.legend()
    plt.show()

    plt.figure()
    for i in range(n_robots):
        plt.plot(target[i][:, 0], target[i][:, 2], label='Estimated y robot ' + str(i))
        plt.plot(target[i][:, 0], target[i][:, 0]*0 + 2, markersize=0)
    plt.xlabel('Y [m]')
    plt.ylabel('Time [s]')
    plt.title('Target y coordinate estimation')
    plt.legend()
    plt.show()

    plt.figure()
    for i in range(n_robots):
        plt.plot(target[i][:, 0], target[i][:, 3], label='Estimated z robot ' + str(i))
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
        plt.hist(loc[i][delay:,3] - gt[i][start + delay:,3], bins=120)
        plt.xlabel('Error [rad]')
        plt.ylabel('Frequency')
        plt.title('Theta localization error in robot ' + str(i))
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

        # Computation of target coordinate mean
        x_mean = np.mean(target[i][:,1])
        y_mean = np.mean(target[i][:,2])
        z_mean = np.mean(target[i][:,3])

        # Computation of target coordinate std
        x_std = np.std(target[i][:,1])
        y_std = np.std(target[i][:,2])
        z_std = np.std(target[i][:,3])

        # Extraction of expected target coordinate std
        expected_x_std = np.mean(target_std[i][:,1])
        expected_y_std = np.mean(target_std[i][:,2])
        expected_z_std = np.mean(target_std[i][:,3])

        # Extraction of expected state estimate std
        loc_x_std = np.mean(loc_std[i][delay:,1])
        loc_y_std = np.mean(loc_std[i][delay:,2])
        loc_theta_std = np.mean(loc_std[i][delay:,3])

        # Computation of actual state estimate std
        real_loc_x_std = np.mean(np.std(gt[i][delay + start:,1] - loc[i][delay:,1]))
        real_loc_y_std = np.mean(np.std(gt[i][delay + start:,2] - loc[i][delay:,2]))
        real_loc_theta_std = np.mean(np.std(gt[i][delay + start:,3] - loc[i][delay:,3]))

        print('Robot ' + str(i) + ":")
        print("X coordinate: " + str(x_mean) + " +- " + str(x_std) + ". Expected std: " + str(expected_x_std))
        print("Y coordinate: " + str(y_mean) + " +- " + str(y_std) + ". Expected std: " + str(expected_y_std))
        print("Z coordinate: " + str(z_mean) + " +- " + str(z_std) + ". Expected std: " + str(expected_z_std))
        print("")

        print('Estimated standard deviations:')
        print("sigma_x: " + str(loc_x_std))
        print("sigma_y: " + str(loc_x_std))
        print("sigma_theta: " + str(loc_theta_std))
        print("")

        print('Actual standard deviations:')
        print("sigma_x: " + str(real_loc_x_std))
        print("sigma_y: " + str(real_loc_x_std))
        print("sigma_theta: " + str(real_loc_theta_std))
        print("")