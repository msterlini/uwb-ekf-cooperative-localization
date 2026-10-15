# Multi-robot UWB localization and cooperative target estimation

[![CI](https://github.com/msterlini/Distributed_project/actions/workflows/ci.yml/badge.svg)](https://github.com/msterlini/Distributed_project/actions/workflows/ci.yml)
![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-22314E)
![Gazebo 11](https://img.shields.io/badge/Gazebo-11-orange)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

A ROS/Gazebo simulation of a team of unicycle robots in an indoor room. Each robot localizes itself with an **Extended Kalman Filter** that fuses wheel odometry with **UWB ranges** to anchors on the ceiling, then searches a target with its camera. The robots that reach the target share their pose and uncertainty, and every robot fuses them into a **3-D estimate of the target position**.

Project of the course *Distributed Estimation for Robots and Vehicles* (2024). The full report, *Distributed unicycle system for the estimation of the 3D position of a target*, is in [`docs/report.pdf`](docs/report.pdf).

## How it works

```mermaid
flowchart LR
    subgraph robot_i["robot i (one namespace per robot)"]
        GZ[Gazebo<br/>odometry, camera, ground truth] -->|odom| KF[kalman_localization<br/>EKF]
        GZ -->|ground_truth| UWB[uwb_dist_sim<br/>noisy ranges]
        UWB -->|uwb_data_topic| KF
        GZ -->|camera| IP[image_processing<br/>yellow blob detection]
        IP -->|target/point_blob| MP[motion_planner<br/>search, approach, stop]
        KF -->|init_move| MP
        MP -->|cmd_vel| GZ
        MP -->|target_height| TE[target_estimator]
        KF -->|localization_data_topic| TE
    end
    TE <-->|/processing_data<br/>pose + uncertainty of every robot| OTHERS[other robots]
    TE -->|target_estimate| OUT((x, y, z ± σ))
```

**Ego-localization.** The state of each robot is $(x, y, \theta)$. The prediction step uses the unicycle model driven by the odometry increments; the correction step uses the distances $d_j = \lVert p - a_j \rVert$ to the UWB anchors $a_j$, whose noise grows with the distance ($\sigma_j = \sigma_0 + k\, d_j$). The ranges carry no information on $\theta$ while the robot is still, so the heading is first taken from the travelled direction and then corrected only while the robot moves.

**Target estimation.** A robot that reached the target is stopped and points at it, so the target lies on its bearing line $-\sin\theta_i\, x + \cos\theta_i\, y = -\sin\theta_i\, x_i + \cos\theta_i\, y_i$. The planar position is the weighted least-squares intersection of the lines of all the robots, each weighted with the uncertainty of the robot pose propagated to the line; the height comes from the elevation at which every camera sees the target.

## Results

Headless run of 5 robots and 6 anchors, evaluated against the Gazebo ground truth with [`plots/scripts/uncertainty.py`](plots/scripts/uncertainty.py):

| Robot | RMSE x [m] | RMSE y [m] | RMSE theta [deg] | Mean predicted std x, y [m] |
|---|---|---|---|---|
| 0 | 0.043 | 0.042 | 10.85 | 0.020, 0.021 |
| 1 | 0.016 | 0.021 | 8.26 | 0.021, 0.016 |
| 2 | 0.013 | 0.015 | 2.51 | 0.007, 0.021 |
| 3 | 0.036 | 0.038 | 19.54 | 0.019, 0.023 |
| 4 | 0.022 | 0.020 | 5.70 | 0.026, 0.015 |

The position errors are a few centimetres and consistent with the covariance predicted by the filter. The heading error stays constant after the initial estimate from the travelled direction, without drifting. With the two robots that reached the target, the cooperative estimate was (0.91, 1.74, 0.83) m against the true (1.00, 2.00, 0.80) m.

## Running the simulation

The simulation opens Gazebo, the camera of a robot with the detected target and PlotJuggler (layouts in [`plotjuggler/`](plotjuggler)). It can run natively, in Docker or in the browser.

### Locally with ROS Noetic (Ubuntu 20.04)

```sh
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone https://github.com/msterlini/Distributed_project.git dist_project
cd ~/catkin_ws
rosdep update --include-eol-distros
rosdep install --from-paths src --ignore-src -y
catkin_make
source devel/setup.bash
roslaunch dist_project init.launch
```

### With Docker

No ROS installation needed, only Docker:

```sh
git clone https://github.com/msterlini/Distributed_project.git
cd Distributed_project
docker compose up web          # desktop in the browser: http://localhost:6080/vnc.html (any OS)
```

On a Linux host with an X server the windows can also open directly on the desktop:

```sh
xhost +local:docker
docker compose up sim
```

### In the browser with GitHub Codespaces

[![Open in GitHub Codespaces](https://github.com/codespaces/badge.svg)](https://codespaces.new/msterlini/Distributed_project)

Once the codespace is built, run in its terminal:

```sh
vnc.sh roslaunch dist_project init.launch
```

and open the forwarded port 6080 (`/vnc.html`). Gazebo renders on the CPU there, so the simulation runs slower than real time.

### Headless runs and results

Record a bag in `bagfiles/` without any window, then compute the error statistics and the figures in `plots/`:

```sh
# Docker
DURATION=300 BAG_NAME=my_run docker compose run --rm headless
docker compose run --rm -v ./plots:/catkin_ws/src/dist_project/plots headless \
    python3 src/dist_project/plots/scripts/uncertainty.py src/dist_project/bagfiles/my_run.bag --save

# Native
roslaunch dist_project init.launch gui:=false record:=true bag_name:=my_run
python3 plots/scripts/uncertainty.py bagfiles/my_run.bag --save
```

Launch arguments of `init.launch`: `gui` (default `true`), `plotjuggler` (default `true`, set it to `false` if PlotJuggler is not installed), `record`, `bag_name`, `overview_camera` (fixed camera above the room on `/overview_camera/image_raw`). All the parameters of the filter and of the sensors are in [`launch/init.launch`](launch/init.launch).

### Changing the scenario

The number and the positions of robots, anchors and target are read from the csv files in [`coordinates/`](coordinates): adding a line adds a robot or an anchor, no code changes needed.

### Tests

The estimation code is plain Python/NumPy in [`src/dist_project`](src/dist_project) and is tested without ROS:

```sh
pip install numpy pytest
pytest
```

## Repository structure

| Folder | Content |
|---|---|
| `scripts/` | ROS nodes, see [scripts/README.md](scripts/README.md) |
| `src/dist_project/` | EKF, bearing-only triangulation and anchor lookup |
| `launch/` | `init.launch` (whole simulation), `start_nodes.launch` (nodes of one robot) |
| `coordinates/` | spawn positions of robots, anchors and target |
| `urdf/`, `mesh/`, `models/`, `worlds/` | robot description, room, target and Gazebo world |
| `msg/` | custom messages `uwb_data` and `robot_data` |
| `bagfiles/` | recorded simulations |
| `plots/` | scripts and figures of the results |
| `plotjuggler/` | PlotJuggler layouts |
| `test/` | unit tests |

## Using it as a base

The project was built to be a starting point for other course projects: the scene is generated from the csv files, every robot runs in its own namespace and the estimation code is separated from ROS. If you use it, feel free to open an issue to let me know.
