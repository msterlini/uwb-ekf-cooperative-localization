# ROS 2 Jazzy + Gazebo Harmonic environment for the simulation
FROM osrf/ros:jazzy-desktop-full

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pytest \
        ros-jazzy-topic-tools \
        xvfb \
        x11vnc novnc websockify fluxbox \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_WS=/ros2_ws
WORKDIR ${ROS_WS}

COPY . src/dist_project
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install

RUN echo "source ${ROS_WS}/install/setup.bash" >> /root/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
COPY docker/xvfb.sh docker/vnc.sh /usr/local/bin/
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "dist_project", "sim.launch.py"]
