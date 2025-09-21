ARG ROS_DISTRO=humble
FROM ghcr.io/sloretz/ros:${ROS_DISTRO}-desktop

# Environment setup
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Install system packages
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    # Basic tools
    wget curl git nano htop tree sudo bash-completion \
    # ROS packages
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-demo-nodes-cpp \
    ros-${ROS_DISTRO}-demo-nodes-py \
    # Python tools
    python3-pip python3-pytest python3-pytest-cov \
    && rm -rf /var/lib/apt/lists/*

# Create non-root user
RUN groupadd --gid 1000 rosdev \
    && useradd --uid 1000 --gid 1000 -m rosdev \
    && echo "rosdev ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/rosdev \
    && chmod 0440 /etc/sudoers.d/rosdev

# Create workspace structure as root, then change ownership
RUN mkdir -p /home/rosdev/ros_ws/{src/thirdparty_packages,src/host_packages,bags,maps,results,scripts,log} \
    && mkdir -p /home/rosdev/workspace_host \
    && chown -R rosdev:rosdev /home/rosdev

# Switch to rosdev user
USER rosdev
WORKDIR /home/rosdev/ros_ws

# Copy scripts and packages.repos
COPY --chown=rosdev:rosdev scripts/ /home/rosdev/ros_ws/scripts/
COPY --chown=rosdev:rosdev packages.repos /home/rosdev/ros_ws/

# Make scripts executable
RUN chmod +x /home/rosdev/ros_ws/scripts/*.sh

ENV DEBIAN_FRONTEND=dialog