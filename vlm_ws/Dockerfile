FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Basic tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    ca-certificates \
    sudo \
    wget \
    git \
    build-essential \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Locale
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# ROS2 apt repo
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list

# Install ROS2 Humble and TurtleBot3/Gazebo packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    ros-humble-turtlebot3-gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# rosdep init/update (if allowed)
RUN rosdep init || true
RUN rosdep update || true

# pip packages
RUN pip3 install -U pip setuptools
RUN pip3 install argcomplete

# Create non-root user
RUN useradd -ms /bin/bash ros && echo "ros:ros" | chpasswd && adduser ros sudo

# Set environment
ENV ROS_DISTRO=humble
ENV TURTLEBOT3_MODEL=burger

# Ensure ros environment is sourced for the ros user shells
RUN echo "source /opt/ros/humble/setup.bash" >> /home/ros/.bashrc
RUN echo "export TURTLEBOT3_MODEL=burger" >> /home/ros/.bashrc

# Create workspace dir for mounting the host workspace
RUN mkdir -p /home/ros/vlm_ws
RUN chown -R ros:ros /home/ros/vlm_ws

WORKDIR /home/ros
USER ros

# Add entrypoint
COPY docker_entrypoint.sh /usr/local/bin/docker_entrypoint.sh
RUN chmod +x /usr/local/bin/docker_entrypoint.sh
ENTRYPOINT ["/usr/local/bin/docker_entrypoint.sh"]
CMD ["bash"]
