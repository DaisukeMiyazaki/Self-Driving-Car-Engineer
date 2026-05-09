FROM ros:noetic-robot

ARG CARLA_VERSION=0.9.15
ARG BRIDGE_TAG=0.9.15

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    python3-pip \
    python3-rosdep \
    python3-catkin-tools \
    ros-noetic-derived-object-msgs \
    ros-noetic-ackermann-msgs \
    ros-noetic-tf \
    ros-noetic-cv-bridge \
    ros-noetic-pcl-ros \
    && rm -rf /var/lib/apt/lists/*

# carla python API matching the server version
RUN pip3 install --no-cache-dir "carla==${CARLA_VERSION}" \
    "numpy<2" \
    transforms3d \
    pygame \
    networkx \
    simple-pid

# carla_ros_bridge from source (no published noetic deb for 0.9.15)
WORKDIR /opt/carla-ros-bridge
RUN git clone --depth 1 --branch ${BRIDGE_TAG} --recurse-submodules \
    https://github.com/carla-simulator/ros-bridge.git src

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd /opt/carla-ros-bridge && \
    catkin config --init --extend /opt/ros/noetic --install && \
    catkin build"

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
