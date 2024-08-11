# Distributed project

The aim of this project was to develop a environment in ROS and Gazebo for the deployment of a indoor scenario with many unicycles that could be used as a starting basis for future projects.

In particular in order to address the topics of the course this project saw the dynamic deployment of a number of unicycles and UWB tags, the problems that were solved regard the ego-localization and the collaborative estimation of the position of a target in the 3-D environment.

## Project structure

This project follows the catkin workspace folder hierarchy as recommended by ROS. For more information on the catkin workspace structure, please refer to the [ROS wiki](http://wiki.ros.org/catkin/workspaces).

In particular the main folders where most of the material lie are the following, in each folder there will be further information:

- `bagfiles/`: Where simulation bagfiles are stored.
- `coordinates/`: Here there are the csv files containing the initial spawn coordinates of the UWB tags, robots and targets. These files can be edited to dynamically change the number and position of the entities of the simulation.
- `launch/`: ROS launch files to initialize the simulation.
- `mesh/`: Contains the stl files used to render the robot in the Gazebo simulation.
- `models/`: Contains the models folders created in Gazebo used to spawn the room and the target in the simulation.
- `msg/`: Custom ROS messages definition.
- `plotjuggler/`: Contains the settings file for the third party node "plotjuggler" used to better plot data from the topics both from bag file and in real time.
- `plots/`: Contains the scripts used to create the plots that have been used in the final report.
- `scripts/`: Here are all the executable python scripts, they will manage the setup of the environment and all the logic of the ROS nodes
- `urdf/`: URDF files for robot description. 


## Requirements

- Docker installed

## Installation 
There are many requirements necessary, in order to simplify the execution the whole project has been developed inside a Docker container in order to guarantee compatibility along all platforms. 

This github repository must be cloned into a folder on the user computer with:
```sh
git clone https://github.com/Marcosterlo/Distributed_project.git
```


To run the Docker container for this project, use the following command. You will need to replace `/path/to/your/local/folder` with the local folder where this GitHub repository has been cloned to.

```sh
sudo docker run --name ubuntu_ros -v /tmp/.X11-unix/:/tmp/.X11-unix/ -v /path/to/your/local/folder:/home/marco/shared --env="DISPLAY=$DISPLAY" --privileged --shm-size 2g --rm -i -t --user=marco --workdir=/home/marco marcosterlo/ubuntu:ros bash
```

## Usage

Once inside the docker container, in order to start the simulation run the following command:

```sh
roslaunch dist_project init.launch
```
It will also open a window containing the point of view of the camera of one of the unicycles
