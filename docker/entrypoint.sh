#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source "${ROS_WS}/install/setup.bash"
exec "$@"
