# Build arguments
ARG ROS_DISTRO=humble

# Base image
FROM ros:${ROS_DISTRO}-ros-base

# An ARG declared before a FROM is outside of a build stage, so it can’t be used in any instruction after a FROM

# Install additional ros packages and other libraries
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get upgrade -yq && apt-get install -y \
    git \
    make \
    && rm -rf /var/lib/apt/lists/*

# Install wiring-pi
RUN git clone https://github.com/WiringPi/WiringPi && cd WiringPi && ./build

# Add copy of local workspace and setup script
WORKDIR /home/user/ros/TactileDrone
ADD ws/ ./ws/
WORKDIR /home/user/ros/TactileDrone/ws

# Setup workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --packages-select px4_msgs px4_ros_com
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --packages-select custom_interfaces
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --packages-select sensors_and_observers
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --packages-select planners

# Add the entrypoint script
ADD entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
