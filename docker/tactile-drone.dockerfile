# Build arguments
ARG ROS_DISTRO=humble

# Base image
FROM ros:${ROS_DISTRO}-ros-base

# An ARG declared before a FROM is outside of a build stage, so it can’t be used in any instruction after a FROM

# Install additional ros packages and other libraries
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get upgrade -yq && apt-get install -y \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    git \
    make \
    && rm -rf /var/lib/apt/lists/*

# Install wiring-pi
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get upgrade -yq && apt-get install -y wiringpi libwiringpi-dev

# Add copy of local workspace and setup script
WORKDIR /home/user/ros/TactileDrone
ADD ws/ ./ws/
WORKDIR /home/user/ros/TactileDrone/ws

# Setup workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build

# Add the entrypoint script
ADD docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh" ]
