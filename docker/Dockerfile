############################################################
# NIDAR AirMouse Development Environment
#
# Ubuntu 20.04
# ROS Noetic
# Fast-Planner
# FUEL
# FAST-LIO2
# PX4/MAVROS Ready
############################################################

FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

############################################################
# Basic Linux packages
############################################################

RUN apt-get update && apt-get install -y \
    apt-utils \
    software-properties-common \
    curl \
    wget \
    git \
    gnupg2 \
    lsb-release \
    sudo \
    nano \
    vim \
    unzip \
    build-essential \
    cmake \
    pkg-config \
    python3-pip \
    python3-dev \
    python3-setuptools \
    python3-wheel \
    net-tools \
    iputils-ping \
    htop \
    && rm -rf /var/lib/apt/lists/*

############################################################
# ROS Noetic Repository
############################################################

RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
    > /etc/apt/sources.list.d/ros1.list

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

############################################################
# ROS Installation
############################################################

RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-catkin-tools \
    python3-osrf-pycommon \
    && rm -rf /var/lib/apt/lists/*

############################################################
# Initialize rosdep
############################################################

RUN rosdep init

############################################################
# Fast-Planner / FUEL dependencies
############################################################

RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libarmadillo-dev \
    libdw-dev \
    libsuitesparse-dev \
    libyaml-cpp-dev \
    libopencv-dev \
    libpcl-dev \
    libnlopt-dev \
    libnlopt-cxx-dev \
    && rm -rf /var/lib/apt/lists/*

############################################################
# MAVROS
############################################################

RUN apt-get update && apt-get install -y \
    ros-noetic-mavros \
    ros-noetic-mavros-extras \
    && rm -rf /var/lib/apt/lists/*

############################################################
# TF / Navigation / Common ROS packages
############################################################

RUN apt-get update && apt-get install -y \
    ros-noetic-tf2 \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-eigen \
    ros-noetic-image-transport \
    ros-noetic-cv-bridge \
    ros-noetic-pcl-ros \
    ros-noetic-octomap \
    ros-noetic-octomap-msgs \
    ros-noetic-octomap-ros \
    ros-noetic-joy \
    ros-noetic-rviz \
    && rm -rf /var/lib/apt/lists/*

############################################################
# OpenGL Support
############################################################

RUN apt-get update && apt-get install -y \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libglu1-mesa \
    libx11-6 \
    libxext6 \
    libxrender1 \
    libxtst6 \
    && rm -rf /var/lib/apt/lists/*

############################################################
# Create Developer User
############################################################

ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd -m -s /bin/bash \
    --uid ${USER_UID} \
    --gid ${USER_GID} \
    ${USERNAME}

RUN echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

############################################################
# Switch User
############################################################

USER ${USERNAME}

WORKDIR /home/${USERNAME}

############################################################
# Catkin Workspace
############################################################

RUN mkdir -p ~/catkin_ws/src

############################################################
# ROS Environment
############################################################

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

############################################################
# Default Shell
############################################################

CMD ["/bin/bash"]
