# Bagfiles

This folder contains ROS bagfiles, which are used for recording and playing back ROS message data.

## What is a ROS Bagfile?

A ROS bagfile (with a `.bag` extension) is a file format in ROS for storing ROS message data. Bagfiles are used to record the output of ROS topics and can be played back later, making them useful for data analysis, debugging, and testing. They capture the exact state of the topics being published at the time of recording, allowing for consistent reproduction of the recorded events.

## Usage in this project

ROS bagfiles have been used to store simulations' runs in order to manipulate the data and plot the desired features to add in the report.

The following commands have been used to acquire the files:

```sh
rosbag record /robot0/localization_data_topic /robot0/ground_truth/state -O few_tags.bag
```
```sh
rosbag record -e "(.*)localization_data_topic(.*)" "(.*)ground_truth/state(.*)" "(.*)target_estimate(.*)" -O total_bag.bag
```