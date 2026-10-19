import csv
import os
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib
import numpy as np
from rosbags.highlevel import AnyReader

# Without a display the figures are saved
SAVE = '--save' in sys.argv or not os.environ.get('DISPLAY')
if SAVE:
    matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

PACKAGE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')

# Height of the center of the yellow sphere once the target lies on the floor
TARGET_Z = 0.8
# Position std [m] below which the filter is converged
CONVERGED_STD = 0.2


def read_coordinates(name):
    with open(os.path.join(PACKAGE_DIR, 'coordinates', name)) as f:
        return np.array([[float(x) for x in row] for row in csv.reader(f) if row])


def yaw(q):
    # Rotation around z of the quaternion
    return np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read_bag(path):
    # Messages of every topic as (header time [s], message), from ROS 1 or ROS 2 bags
    messages = defaultdict(list)
    with AnyReader([Path(path)]) as reader:
        for connection, timestamp, rawdata in reader.messages():
            msg = reader.deserialize(rawdata, connection.msgtype)
            stamp = msg.header.stamp
            sec = stamp.sec if hasattr(stamp, 'sec') else stamp.secs
            nanosec = stamp.nanosec if hasattr(stamp, 'nanosec') else stamp.nsecs
            messages[connection.topic].append((sec + nanosec * 1e-9, msg))
    return messages


def show(name=None):
    if SAVE:
        if name is not None:
            plt.savefig(os.path.join(PACKAGE_DIR, 'plots', name + '.png'), dpi=120, bbox_inches='tight')
        plt.close()
    else:
        plt.show()


if __name__ == "__main__":

    # Path of the bag to extract data from, bagfiles/total_bag by default
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    bagfile = args[0] if args else os.path.join(PACKAGE_DIR, 'bagfiles', 'total_bag')

    # Positions of tags and target and number of robots of the simulation taken from the csv files
    tags = read_coordinates('tag_coordinates.csv')
    target_gt = np.append(read_coordinates('target_coordinates.csv')[0], TARGET_Z)
    n_robots = len(read_coordinates('robot_coordinates.csv'))

    # Lists of total data
    gt = []
    loc = []
    loc_std = []
    target = []
    target_std = []

    messages = read_bag(bagfile)
    for i in range(n_robots):
        id = "/robot" + str(i)

        # Ground truth data extraction
        tmp = []
        for t, msg in messages[id + "/ground_truth/state"]:
            pose = msg.pose.pose
            tmp.append([t, pose.position.x, pose.position.y, yaw(pose.orientation)])
        gt.append(np.array(tmp))

        # Localization data extraction
        tmp1 = []
        tmp2 = []
        for t, msg in messages[id + "/localization_data_topic"]:
            pose = msg.pose.pose
            cov = np.reshape(msg.pose.covariance, (6, 6))
            # I skip the estimates before the first convergence
            if not tmp1 and np.sqrt(cov[0, 0] + cov[1, 1]) > CONVERGED_STD:
                continue
            tmp1.append([t, pose.position.x, pose.position.y, yaw(pose.orientation)])
            tmp2.append([t, np.sqrt(cov[0, 0]), np.sqrt(cov[1, 1]), np.sqrt(cov[5, 5])])
        loc.append(np.array(tmp1))
        loc_std.append(np.array(tmp2))

        # Target estimate data extraction
        tmp1 = []
        tmp2 = []
        for t, msg in messages[id + "/target_estimate"]:
            pose = msg.pose.pose
            cov = np.reshape(msg.pose.covariance, (6, 6))
            tmp1.append([t, pose.position.x, pose.position.y, pose.position.z])
            tmp2.append([t, np.sqrt(cov[0, 0]), np.sqrt(cov[1, 1]), np.sqrt(cov[2, 2])])
        target.append(np.array(tmp1).reshape(-1, 4))
        target_std.append(np.array(tmp2).reshape(-1, 4))

    # The time starts from the first localization message
    t0 = min(loc[i][0, 0] for i in range(n_robots))

    # Seconds to wait for the filter to converge before computing the errors
    delay = 10

    # Localization errors computed interpolating the ground truth at the time of each estimate
    errors = []
    for i in range(n_robots):
        gt_x = np.interp(loc[i][:, 0], gt[i][:, 0], gt[i][:, 1])
        gt_y = np.interp(loc[i][:, 0], gt[i][:, 0], gt[i][:, 2])
        gt_theta = np.interp(loc[i][:, 0], gt[i][:, 0], np.unwrap(gt[i][:, 3]))
        error = np.column_stack((loc[i][:, 1] - gt_x, loc[i][:, 2] - gt_y, loc[i][:, 3] - gt_theta))
        error[:, 2] = (error[:, 2] + np.pi) % (2 * np.pi) - np.pi
        errors.append(error[loc[i][:, 0] > loc[i][0, 0] + delay])

    # Data plotting
    plt.figure()
    for i in range(n_robots):
        line, = plt.plot(loc[i][:, 1], loc[i][:, 2], label='Estimate robot ' + str(i))
        plt.plot(gt[i][:, 1], gt[i][:, 2], '--', color=line.get_color(), alpha=0.6)
    plt.plot(tags[:, 0], tags[:, 1], label='UWB tags', markersize=12, linestyle='None', marker='*', color='blue')
    plt.plot(target_gt[0], target_gt[1], label='Target', markersize=12, linestyle='None', marker='o', color='gold')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('Multiple localization (dashed: ground truth)')
    plt.axis('equal')
    plt.legend(fontsize='small')
    show('multiple')

    for k, coordinate in enumerate(['x', 'y', 'z']):
        plt.figure()
        for i in range(n_robots):
            if len(target[i]):
                plt.plot(target[i][:, 0] - t0, target[i][:, k + 1], label='Estimated ' + coordinate + ' robot ' + str(i))
        plt.axhline(target_gt[k], color='black', linestyle='--', label='Ground truth')
        plt.xlabel('Time [s]')
        plt.ylabel(coordinate.upper() + ' [m]')
        plt.title('Target ' + coordinate + ' coordinate estimation')
        plt.legend(fontsize='small')
        show(coordinate + '_estimate')

    if not SAVE:
        for i in range(n_robots):
            for k, (name, unit) in enumerate([('X', 'm'), ('Y', 'm'), ('Theta', 'rad')]):
                plt.figure()
                plt.hist(errors[i][:, k], bins=120)
                plt.xlabel('Error [' + unit + ']')
                plt.ylabel('Frequency')
                plt.title(name + ' localization error in robot ' + str(i))
                show()

    # Summary table in markdown
    print('| Robot | RMSE x [m] | RMSE y [m] | RMSE theta [deg] | Mean predicted std x, y [m] | Final target error [m] |')
    print('|---|---|---|---|---|---|')
    for i in range(n_robots):
        rmse = np.sqrt(np.mean(errors[i]**2, axis=0))
        start = loc_std[i][:, 0] > loc_std[i][0, 0] + delay
        predicted = np.mean(loc_std[i][start, 1:3], axis=0)
        if len(target[i]):
            target_error = '%.3f' % np.linalg.norm(target[i][-1, 1:4] - target_gt)
        else:
            target_error = '-'
        print('| %d | %.3f | %.3f | %.2f | %.3f, %.3f | %s |'
              % (i, rmse[0], rmse[1], np.degrees(rmse[2]), predicted[0], predicted[1], target_error))

    # Final target estimate with uncertainty
    for i in range(n_robots):
        if len(target[i]):
            print('Robot ' + str(i) + ' target estimate: ' + str(np.round(target[i][-1, 1:4], 3))
                  + ' +- ' + str(np.round(target_std[i][-1, 1:4], 3)) + ', ground truth: ' + str(target_gt))
