# Multi-robot UWB localization and cooperative target estimation

[![CI](https://github.com/msterlini/uwb-ekf-cooperative-localization/actions/workflows/ci.yml/badge.svg)](https://github.com/msterlini/uwb-ekf-cooperative-localization/actions/workflows/ci.yml)
![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-22314E)
![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-orange)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

Five unicycle robots in a room localize themselves with an **Extended Kalman Filter** that fuses wheel odometry with **UWB ranges** to anchors on the ceiling, search a target with their cameras and fuse their bearings into a **3-D estimate of the target position**.

Project of the course *Distributed Estimation for Robots and Vehicles* (2024), report in [`docs/report.pdf`](docs/report.pdf). The original ROS 1 Noetic / Gazebo Classic version is on the [`ros1`](https://github.com/msterlini/uwb-ekf-cooperative-localization/tree/ros1) branch.

![Simulation](docs/demo.gif)

## How it works

**Localization.** The state of each robot is $(x, y, \theta)$. The prediction uses the unicycle model driven by the odometry increments, the correction the distances to the UWB anchors, with a noise that grows with the distance. The ranges say nothing about $\theta$ while the robot is still, so the heading is first taken from the travelled direction and then corrected only while the robot moves.

**Target estimation.** A robot that reached the target points at it, so the target lies on its bearing line. The planar position is the weighted least-squares intersection of the lines of all the robots, weighted with the pose uncertainty of each robot; the height comes from the elevation at which each camera sees the target.

![How it works](docs/animation.gif)

## Results

Reference run in [`bagfiles/total_bag`](bagfiles), evaluated against the Gazebo ground truth with [`plots/scripts/uncertainty.py`](plots/scripts/uncertainty.py):

| Robot | RMSE x [m] | RMSE y [m] | RMSE θ [deg] | Predicted σ x, y [m] |
|---|---|---|---|---|
| 0 | 0.015 | 0.016 | 1.1 | 0.017, 0.020 |
| 1 | 0.017 | 0.016 | 0.9 | 0.022, 0.013 |
| 2 | 0.011 | 0.014 | 0.9 | 0.011, 0.022 |
| 3 | 0.020 | 0.022 | 3.1 | 0.017, 0.022 |
| 4 | 0.021 | 0.016 | 3.1 | 0.022, 0.016 |

The target estimate is **(0.987, 2.026, 0.902) m** against the true (1.00, 2.00, 0.80) m: 3 cm on the plane, 10 cm in height. The errors are consistent with the covariance of the filter.

<p align="center">
  <img src="plots/multiple.png" width="49%" />
  <img src="plots/z_estimate.png" width="49%" />
</p>

## Running

With Docker, no ROS installation needed:

```sh
git clone https://github.com/msterlini/uwb-ekf-cooperative-localization.git
cd uwb-ekf-cooperative-localization
./run.sh            # Linux: Gazebo and live monitor on the desktop, GPU detected automatically
./run.sh web        # any OS: live monitor in the browser at http://localhost:6080/vnc.html
.\run.ps1           # Windows PowerShell, same as web
```

Gazebo shows the cameras of the robots with the detected target and their fields of view; the live monitor shows true and estimated trajectories, covariance ellipses, target estimate and errors.

![Live monitor](docs/live_monitor.png)

Natively on Ubuntu 24.04 with ROS 2 Jazzy:

```sh
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/msterlini/uwb-ekf-cooperative-localization.git dist_project
cd ~/ros2_ws && rosdep install --from-paths src --ignore-src -y && colcon build
source install/setup.bash
ros2 launch dist_project sim.launch.py
```

Headless run and analysis:

```sh
DURATION=300 BAG_NAME=my_run ./run.sh headless     # or: ros2 launch dist_project sim.launch.py gui:=false record:=true bag_name:=my_run
pip install rosbags matplotlib
python3 plots/scripts/uncertainty.py bagfiles/my_run --save
```

Launch arguments: `gui`, `gazebo_gui`, `monitor`, `record`, `bag_name`, `bag_dir`, `overview_camera`, `camera_rate` (default 10 Hz: Gazebo Harmonic renders the cameras in lockstep with the physics). Filter and sensor parameters are in [`config/params.yaml`](config/params.yaml), robots, anchors and target in the csv files of [`coordinates/`](coordinates). The estimation code is plain NumPy and is tested without ROS with `pytest`.

## Structure

| Folder | Content |
|---|---|
| `scripts/` | ROS nodes, see [scripts/README.md](scripts/README.md) |
| `src/dist_project/` | EKF, triangulation, anchor lookup |
| `launch/`, `config/`, `coordinates/` | launch file, parameters, scenario |
| `urdf/`, `mesh/`, `models/`, `worlds/` | robot, room and Gazebo world |
| `msg/` | messages `UwbData` and `RobotData` |
| `bagfiles/`, `plots/` | recorded runs, analysis scripts and figures |
| `docker/`, `test/` | Docker setup, unit tests |
