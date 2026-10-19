# Launch files

This folder contains the ROS 2 launch file of the simulation, written in Python.

- `sim.launch.py`: starts Gazebo, spawns room, anchors, target and the robots of `coordinates/`, bridges the Gazebo topics to ROS and starts the nodes of every robot in its own namespace and the live monitor.

To run it:

```sh
ros2 launch dist_project sim.launch.py
ros2 launch dist_project sim.launch.py --show-args   # list of the arguments
```
