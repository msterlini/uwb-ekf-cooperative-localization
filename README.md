# Multi-robot UWB localization and cooperative target estimation (ROS 1)

[![CI](https://github.com/msterlini/uwb-ekf-cooperative-localization/actions/workflows/ci.yml/badge.svg?branch=ros1)](https://github.com/msterlini/uwb-ekf-cooperative-localization/actions/workflows/ci.yml)
![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-22314E)
![Gazebo 11](https://img.shields.io/badge/Gazebo-11-orange)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

Five unicycle robots in a room localize themselves with an **Extended Kalman Filter** that fuses wheel odometry with **UWB ranges** to anchors on the ceiling, search a target with their cameras and fuse their bearings into a **3-D estimate of the target position**.

Project of the course *Distributed Estimation for Robots and Vehicles* (2024), report in [`docs/report.pdf`](docs/report.pdf). This is the original ROS 1 Noetic / Gazebo Classic version; the current ROS 2 Jazzy / Gazebo Harmonic version is on [`main`](https://github.com/msterlini/uwb-ekf-cooperative-localization).

![Simulation](docs/demo.gif)

## How it works

**Localization.** The state of each robot is $(x, y, \theta)$. The prediction uses the unicycle model driven by the odometry increments, the correction the distances to the UWB anchors, with a noise that grows with the distance. The ranges say nothing about $\theta$ while the robot is still, so the heading is first taken from the travelled direction and then corrected only while the robot moves.

**Target estimation.** A robot that reached the target points at it, so the target lies on its bearing line. The planar position is the weighted least-squares intersection of the lines of all the robots, weighted with the pose uncertainty of each robot; the height comes from the elevation at which each camera sees the target.

![How it works](docs/animation.gif)

## Results

Reference run in [`bagfiles/total_bag.bag`](bagfiles), evaluated against the Gazebo ground truth with [`plots/scripts/uncertainty.py`](plots/scripts/uncertainty.py):

| Robot | RMSE x [m] | RMSE y [m] | RMSE θ [deg] | Predicted σ x, y [m] |
|---|---|---|---|---|
| 0 | 0.016 | 0.014 | 4.9 | 0.017, 0.019 |
| 1 | 0.015 | 0.009 | 1.2 | 0.022, 0.012 |
| 2 | 0.010 | 0.014 | 1.7 | 0.008, 0.021 |
| 3 | 0.013 | 0.013 | 2.4 | 0.015, 0.020 |
| 4 | 0.017 | 0.014 | 5.2 | 0.021, 0.015 |

The target estimate is **(0.965, 1.952, 0.825) m** against the true (1.00, 2.00, 0.80) m, an error of 6.4 cm. The errors are consistent with the covariance of the filter.

<p align="center">
  <img src="plots/multiple.png" width="49%" />
  <img src="plots/z_estimate.png" width="49%" />
</p>

## Running

With Docker, no ROS installation needed:

```sh
git clone -b ros1 https://github.com/msterlini/uwb-ekf-cooperative-localization.git
cd uwb-ekf-cooperative-localization
./run.sh            # Linux: Gazebo, camera of a robot and live monitor on the desktop, GPU detected automatically
./run.sh web        # any OS: desktop in the browser at http://localhost:6080/vnc.html
.\run.ps1           # Windows PowerShell, same as web
```

![Live monitor](docs/live_monitor.png)

Natively on Ubuntu 20.04 with ROS Noetic:

```sh
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone -b ros1 https://github.com/msterlini/uwb-ekf-cooperative-localization.git dist_project
cd ~/catkin_ws && rosdep install --from-paths src --ignore-src -y && catkin_make
source devel/setup.bash
roslaunch dist_project init.launch
```

Headless run and analysis:

```sh
DURATION=300 BAG_NAME=my_run ./run.sh headless     # or: roslaunch dist_project init.launch gui:=false record:=true bag_name:=my_run
pip install rosbags matplotlib
python3 plots/scripts/uncertainty.py bagfiles/my_run.bag --save
```

Launch arguments of `init.launch`: `gui`, `monitor`, `record`, `bag_name`, `overview_camera`; it also holds all the filter and sensor parameters. Robots, anchors and target are read from the csv files of [`coordinates/`](coordinates). The estimation code is plain NumPy and is tested without ROS with `pytest`.

## Structure

| Folder | Content |
|---|---|
| `scripts/` | ROS nodes, see [scripts/README.md](scripts/README.md) |
| `src/dist_project/` | EKF, triangulation, anchor lookup |
| `launch/`, `coordinates/` | launch files and parameters, scenario |
| `urdf/`, `mesh/`, `models/`, `worlds/` | robot, room and Gazebo world |
| `msg/` | messages `uwb_data` and `robot_data` |
| `bagfiles/`, `plots/` | recorded runs, analysis scripts and figures |
| `docker/`, `test/` | Docker setup, unit tests |
