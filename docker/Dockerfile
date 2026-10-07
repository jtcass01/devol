# devol simulation image: ROS 2 Lyrical + Gazebo Jetty (via the ros_gz vendor packages) on Ubuntu 26.04,
# matching the native WSL setup. Build from the repository root:
#   docker compose -f docker/compose.yaml build
ARG ROS_DISTRO=lyrical
FROM ros:${ROS_DISTRO}

ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO} \
    DEBIAN_FRONTEND=noninteractive \
    WS=/opt/devol_ws

# Same package set as the native install (apt-mark showmanual on the WSL distro), plus the Mesa
# d3d12 driver pieces needed to render on the GPU through WSLg.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-desktop \
        ros-${ROS_DISTRO}-ros-gz \
        ros-${ROS_DISTRO}-gz-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers \
        ros-${ROS_DISTRO}-moveit \
        ros-${ROS_DISTRO}-moveit-py \
        ros-${ROS_DISTRO}-moveit-ros-perception \
        ros-${ROS_DISTRO}-octomap-server \
        ros-${ROS_DISTRO}-realsense2-description \
        ros-${ROS_DISTRO}-robotiq-description \
        ros-${ROS_DISTRO}-ur \
        ros-${ROS_DISTRO}-vision-msgs \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-teleop-twist-keyboard \
        ros-dev-tools \
        python3-colcon-common-extensions \
        python3-pip \
        python3-pytest \
        python3-numpy \
        python3-scipy \
        python3-matplotlib \
        python3-opencv \
        python3-pyqt5 \
        mesa-utils \
        libgl1-mesa-dri \
        libegl1 \
        libvulkan1 \
        ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# Resolve any remaining package.xml dependencies, then build the workspace into the image.
# src is copied from the checkout as it is on disk, uncommitted changes included.
WORKDIR ${WS}
COPY src ${WS}/src
RUN apt-get update \
    && (rosdep init 2>/dev/null || true) \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} \
    && rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

COPY docker/entrypoint.sh /entrypoint.sh
RUN sed -i 's/\r$//' /entrypoint.sh && chmod +x /entrypoint.sh \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash && source ${WS}/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
