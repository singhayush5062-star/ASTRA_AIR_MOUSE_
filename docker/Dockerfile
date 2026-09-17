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
#
# psmisc supplies killall, which scripts/test_takeoff.sh uses to clear the
# previous run. Without it that cleanup line fails silently (stderr is
# discarded and `|| true` swallows the error) and every re-run inherits the
# old rosmaster/gzserver/px4/mavros_node processes.
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
    psmisc \
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
# PX4 SITL (Gazebo Classic) build dependencies
#
# Matches simulation/PX4-Autopilot-v1.14.3/Tools/setup/ubuntu.sh. Without the
# gstreamer -dev packages, sitl_gazebo-classic's CMake configure fails with
# GSTREAMER_APP_LIBRARIES set to NOTFOUND (gazebo11/libgazebo11-dev already
# come in via ros-noetic-desktop-full above).
############################################################

RUN apt-get update && apt-get install -y \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    dmidecode \
    libimage-exiftool-perl \
    && rm -rf /var/lib/apt/lists/*

############################################################
# MAVROS
############################################################

RUN apt-get update && apt-get install -y \
    ros-noetic-mavros \
    ros-noetic-mavros-extras \
    geographiclib-tools \
    && rm -rf /var/lib/apt/lists/*

# MAVROS' UAS node hard-requires the EGM96-5 geoid grid at
# /usr/share/GeographicLib/geoids/egm96-5.pgm; without it mavros aborts as a
# FATAL/REQUIRED process the instant it starts (GeographicLib exception),
# which takes down the whole SITL roslaunch with it.
RUN geographiclib-get-geoids egm96-5

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
# Livox-SDK System Installation
############################################################

RUN git clone https://github.com/Livox-SDK/Livox-SDK.git /tmp/Livox-SDK && \
    cd /tmp/Livox-SDK && \
    rm -rf build && mkdir -p build && cd build && \
    cmake .. && make -j$(nproc) && make install && \
    ldconfig && \
    rm -rf /tmp/Livox-SDK

############################################################
# NVIDIA GPU Runtime Support
#
# These env vars are read by the NVIDIA Container Toolkit on the host when
# the container is launched with `--gpus all` (see scripts/docker_dev_start.sh).
# They are no-ops on a CPU-only host / without --gpus.
############################################################

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics

############################################################
# GPU Inference Stack (PyTorch + Ultralytics/YOLO)
#
# CUDA runtime ships inside the torch wheel itself, so no system CUDA
# toolkit install is required here -- only a host driver + nvidia-container-
# toolkit compatible with CUDA 12.1 (any driver >= 525 works).
############################################################

RUN pip3 install --no-cache-dir \
    torch==2.2.2 torchvision==0.17.2 \
    --index-url https://download.pytorch.org/whl/cu121

# >= 8.3.0 is required for YOLO11/YOLO26 architectures (C3k2 module).
# PERSON_DETECTION_MODEL_V3 was exported with 8.4.120, so 8.4.x is the safe floor.
#
# The `ncnn` package is the CPU inference backend the deployed detector uses
# (see mission_config.yaml, detection.model). Without it ultralytics will
# auto-download and pip-install ncnn on first NCNN load, which is a 100+s
# stall that also fails in an air-gapped competition environment (§7).
RUN pip3 install --no-cache-dir \
    "ultralytics>=8.4.120,<8.5" \
    ncnn

############################################################
# PX4 Autopilot & ROS Python Dependencies
############################################################

RUN pip3 install --no-cache-dir \
    "empy==3.3.4" \
    argcomplete \
    argparse \
    cerberus \
    coverage \
    future \
    jinja2 \
    jsonschema \
    kconfiglib \
    lxml \
    matplotlib \
    numpy \
    nunavut \
    packaging \
    pandas \
    pkgconfig \
    psutil \
    pygments \
    pymavlink \
    pyros-genmsg \
    pyserial \
    pyulog \
    pyyaml \
    requests \
    six \
    toml \
    sympy \
    rospkg \
    catkin_pkg

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
