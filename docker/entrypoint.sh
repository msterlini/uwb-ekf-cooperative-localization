#!/bin/bash
set -e
source /opt/ros/noetic/setup.bash
source "${CATKIN_WS}/devel/setup.bash"
exec "$@"
