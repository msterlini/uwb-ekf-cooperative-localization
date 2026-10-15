# Launch files

This folder contains ROS launch files, which are used to start nodes and set up configurations for running ROS applications.

Launch files have the `.launch` extension and are written in XML format.

- `init.launch`: Main launch file to start the simulation.
- `start_nodes.launch`: Launch file that calls itself recursively to start all the necessary nodes in each robot's namespace

To run a launch file, use the following command:

```sh
roslaunch dist_project launch_file.launch
```