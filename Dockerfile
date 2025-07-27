# Base image
FROM ubuntu:24.04

ENV ROS_DISTRO=jazzy
ENV DEBIAN_FRONTEND=noninteractive

# Create a non-root user
ARG USERNAME=devol-dev
ARG USER_UID
ARG USER_GID

# Install dependencies
RUN apt-get update && apt-get install -y \
    sudo \
    build-essential \
    cmake \
    wget \
    git \
    curl \
    openssh-client \
    openssh-server \
    xauth \
    sshpass \
    unzip \
    ca-certificates \
    gdb \
    wget \
    python3 \
    python3-venv \
    python3-pip \
    python3-setuptools \
    software-properties-common \
    && add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update -y \
    && apt-get upgrade -y \
    && apt-get install -y ros-dev-tools ros-${ROS_DISTRO}-desktop \
    # Ros2 control libraries
    ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers ros-${ROS_DISTRO}-teleop-twist-keyboard \
    # Universal Robots Library
    ros-${ROS_DISTRO}-ur \
    # Gazebo Libraries
    ros-${ROS_DISTRO}-ros-gz ros-${ROS_DISTRO}-gz-ros2-control \
    # Robotiq Libraries
    ros-${ROS_DISTRO}-robotiq-description ros-${ROS_DISTRO}-robotiq-controllers \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd -ms /bin/bash --uid ${USER_UID} --gid ${USER_GID} -G sudo ${USERNAME} \
    && echo "${USERNAME}:${USERNAME}" | chpasswd

# Switch to the non-root user
USER ${USERNAME}

# Upgrade pip and install Python dependencies
RUN python3 -m venv /home/${USERNAME}/.devol_venv \
    && . /home/${USERNAME}/.devol_venv/bin/activate \
    && pip install --upgrade pip setuptools \
    && pip install --upgrade colcon-common-extensions \
    && . /opt/ros/${ROS_DISTRO}/setup.sh

ENV PATH=/home/${USERNAME}/.devol_venv/bin:/home/${USERNAME}/.local/bin:$PATH

COPY --chown=${USERNAME}:${USERNAME} .bashrc /home/${USERNAME}/.bashrc
