# ROS Noetic + Gazebo 11 environment for the simulation
FROM osrf/ros:noetic-desktop-full

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-noetic-plotjuggler-ros \
        python3-pytest \
        xvfb \
        x11vnc novnc websockify fluxbox \
    && rm -rf /var/lib/apt/lists/*

ENV CATKIN_WS=/catkin_ws
WORKDIR ${CATKIN_WS}

COPY . src/dist_project
RUN . /opt/ros/noetic/setup.sh && catkin_make

RUN echo "source ${CATKIN_WS}/devel/setup.bash" >> /root/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
COPY docker/xvfb.sh docker/vnc.sh /usr/local/bin/
ENTRYPOINT ["/entrypoint.sh"]
CMD ["roslaunch", "dist_project", "init.launch"]
