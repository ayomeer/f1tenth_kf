# syntax = docker/dockerfile:1.2
# ^ Enable BuildKit features for apt Cache Mounts

##############################################
# Created from template ros2.dockerfile.jinja
##############################################

###########################################
# Base image -> docker build --target=base
###########################################
FROM ubuntu:20.04 AS base

ARG TARGETPLATFORM
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=galactic

RUN rm -f /etc/apt/apt.conf.d/docker-clean; echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache
RUN \
  --mount=type=cache,id=${TARGETPLATFORM},target=/var/cache/apt,sharing=locked \
  case "$TARGETPLATFORM" in \
  "linux/amd64") \
    apt-get update && apt-get install -y \
    locales \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/* \
    && export LANG=en_US.UTF-8 \
    && ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
    && export DEBIAN_FRONTEND=noninteractive \
    && apt-get update \
    && apt-get install -y tzdata \
    && dpkg-reconfigure --frontend noninteractive tzdata \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get update && apt-get -y upgrade \
    && rm -rf /var/lib/apt/lists/* ;; \  
  "linux/arm64") \
    echo "Building for jetson's arm64 platform" ;; \
  *) \
    echo "Building for unexpected platform!" ;; \
esac;


# Install common programs
RUN \
  --mount=type=cache,id=${TARGETPLATFORM},target=/var/cache/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    sudo \
    software-properties-common \
    wget \
    apt-utils \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2
RUN \
  --mount=type=cache,id=${TARGETPLATFORM},target=/var/cache/apt,sharing=locked \
  sudo add-apt-repository universe \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-base \
    python3-argcomplete \
  && rm -rf /var/lib/apt/lists/*

ENV AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib
ENV PATH=/opt/ros/${ROS_DISTRO}/bin:$PATH
ENV PYTHONPATH=/opt/ros/${ROS_DISTRO}/lib/python3.10/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV DEBIAN_FRONTEND=



##############################################
#  Develop image -> docker build --target=dev
##############################################
FROM base AS dev

ARG TARGETPLATFORM
ENV DEBIAN_FRONTEND=noninteractive

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user "ros"
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  # Add sudo support for the non-root user
  && apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

# Give new user rights to its own home dir
RUN chown $USER_UID:$USER_GID /home/$USERNAME

# Use default bashrc for user
RUN cp /etc/skel/.bashrc /home/$USERNAME

# Create ros_ws with ros user to avoid permission issues
USER ros
RUN mkdir -p ~/ros_ws/src
USER root

# Apt installs, first three lines cache apt installs for future builds
RUN rm -f /etc/apt/apt.conf.d/docker-clean; echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache
RUN \
  --mount=type=cache,id=${TARGETPLATFORM},target=/var/cache/apt,sharing=locked \
  apt-get update && apt-get install -y --no-install-recommends \
  bash-completion \
  # build-essential \
  # cmake \
  # gdb \
  git \
  git-core \
  openssh-client \
  python3-argcomplete \
  python3-pip \
  ros-dev-tools \
  ros-${ROS_DISTRO}-ament-* \
  # ros-${ROS_DISTRO}-xacro \
  # ros-${ROS_DISTRO}-joint-state-publisher \
  # ros-${ROS_DISTRO}-joint-state-publisher-gui \
  ros-${ROS_DISTRO}-demo-nodes-py \
  ros-${ROS_DISTRO}-demo-nodes-cpp \
  # ros-${ROS_DISTRO}-image-transport-plugins \
  # ros-${ROS_DISTRO}-rqt-image-view \
  # ros-${ROS_DISTRO}-v4l2-camera \
  # v4l-utils \
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
  iputils-ping \
  net-tools \
  nano \
  usbutils \
  jstest-gtk \
  evtest \
  && rm -rf /var/lib/apt/lists/*

RUN usermod -aG video $USERNAME 

RUN pip3 install ipython
RUN rosdep init || echo "rosdep already initialized"

# Set up autocompletion for user
RUN \
  echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc \
  && echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /home/$USERNAME/.bashrc \
  && rm -rf /var/lib/apt/lists/* 

# Add CycloneDDS source command to bashrc (actual config mounted with project files on container startup)
# Note: --platform=linux/arm64/v8 resolves to $TARGETPLATFORM=linx/arm64 (!)
RUN \ 
if [ "$TARGETPLATFORM" = "linux/amd64" ]; \
then \
  echo "Setting CycloneDDS config for PC"; \
  echo "source /home/ros/config/cycloneEnv_pc.sh" >> /home/ros/.bashrc; \
elif [ "$TARGETPLATFORM" = "linux/arm64" ]; \
then \
  echo "Setting CycloneDDS config for Jetson"; \
  echo "source /home/ros/config/cycloneEnv_jetson.sh" >> /home/ros/.bashrc; \
else \
  echo "Building for unexpected platform!"; \
fi 

# fix jstest first time launch error
RUN mkdir -p /home/$USERNAME/.config

ENV DEBIAN_FRONTEND=
ENV AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1


############################################
#  Full image -> docker build --target=full
############################################
FROM dev AS full

ENV DEBIAN_FRONTEND=noninteractive
# Install the full release
RUN apt-get update && apt-get install -y --no-install-recommends \
  ros-${ROS_DISTRO}-desktop \
  && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=

# Fix setuptools version
RUN pip3 install setuptools==58.2.0

ENV DEBIAN_FRONTEND=noninteractive
# Install gazebo
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
  && apt-get update && apt-get install -q -y --no-install-recommends \
    ros-${ROS_DISTRO}-gazebo* \
  && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=

# gazebo no audio card error message fix
RUN touch /etc/asound.conf && echo -e " \
pcm.!default { \n\
  type plug \n\
  slave.pcm 'null' \n\
} \n\
EOF ">> /etc/asound.conf

###########################################
#  Full+Gazebo+Nvidia image
###########################################
FROM full AS nvidia

################
# Expose the nvidia driver to allow opengl 
# Dependencies for glvnd and X11.
################
RUN apt-get update \
 && apt-get install -y -qq --no-install-recommends \
  libglvnd0 \
  libgl1 \
  libglx0 \
  libegl1 \
  libxext6 \
  libx11-6

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV QT_X11_NO_MITSHM 1

