# syntax=docker/dockerfile:1

ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-rmw-fastrtps-cpp

RUN apt-get install -y \
    ros-${ROS_DISTRO}-rtabmap-odom \
    ros-${ROS_DISTRO}-rtabmap-slam

RUN apt-get install -y \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-tf-transformations

RUN apt-get install -y \
    ros-${ROS_DISTRO}-rosbag2

RUN apt-get install -y \
    ros-${ROS_DISTRO}-foxglove-bridge

RUN apt-get install -y \
    python3-serial \
    ca-certificates \
    gnupg \
    wget \
    xvfb \
    xauth \
    x11-xserver-utils \
    libgl1 \
    libglib2.0-0 \
    libxrandr2 \
    libxrender1 \
    libxi6 \
    libxkbcommon0 \
    libxkbcommon-x11-0 \
    libxcb-cursor0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-xinerama0 \
    libxcb-xinput0 \
    libfontconfig1 \
    libfreetype6 \
    libegl1 \
    libglu1-mesa \
    libosmesa6 \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Webots + webots_ros2_driver
RUN bash -lc "set -euo pipefail && \
    mkdir -p /etc/apt/keyrings && \
    wget -qO - https://cyberbotics.com/Cyberbotics.asc | gpg --dearmor -o /etc/apt/keyrings/cyberbotics-archive-keyring.gpg && \
    echo 'deb [arch=amd64 signed-by=/etc/apt/keyrings/cyberbotics-archive-keyring.gpg] https://cyberbotics.com/debian/ binary-amd64/' > /etc/apt/sources.list.d/cyberbotics.list && \
    apt-get update && \
    apt-get install -y \
       webots \
       ros-${ROS_DISTRO}-webots-ros2 \
       ros-${ROS_DISTRO}-webots-ros2-driver && \
    rm -rf /var/lib/apt/lists/*"

# Workspace
ENV WS=/workspace
RUN mkdir -p $WS
COPY ./*.py $WS/
COPY ./ros2_ws $WS/ros2_ws

# Сборка
WORKDIR $WS
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

RUN echo source /opt/ros/${ROS_DISTRO}/setup.bash >> /root/.bashrc
RUN echo source /workspace/install/setup.bash >> /root/.bashrc

ENV ROS_DISTRO=${ROS_DISTRO}
# Для headless-режима Webots/Qt
ENV QT_QPA_PLATFORM=offscreen
ENV WEBOTS_HEADLESS=TRUE
ENV WEBOTS_DISABLE_SOUND=TRUE
