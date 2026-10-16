# Launch files

This folder contains the ROS 2 launch file of the simulation, written in Python.

- `sim.launch.py`: starts Gazebo Harmonic with `worlds/room.world`, spawns the room, the UWB anchors, the target and the robots read from the csv files in `coordinates/`, publishes the static transforms of the anchors, bridges the Gazebo topics (commands, odometry, ground truth, cameras, clock) to ROS and starts the nodes of every robot in its own namespace, plus the camera viewer and the live monitor.

To run it:

```sh
ros2 launch dist_project sim.launch.py
ros2 launch dist_project sim.launch.py --show-args   # list of the arguments
```
