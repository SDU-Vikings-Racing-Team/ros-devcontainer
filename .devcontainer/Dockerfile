ARG ROS_DISTRO=humble
ARG DEV_USER=rosdev
ARG DEV_USER_ID=1000
ARG DEV_GROUP_ID=1000

FROM ghcr.io/sloretz/ros:${ROS_DISTRO}-desktop

# Pass build arguments to environment
ARG ROS_DISTRO
ARG DEV_USER
ARG DEV_USER_ID
ARG DEV_GROUP_ID

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO} \
    DEV_USER=${DEV_USER}

SHELL ["/bin/bash", "-c"]

# Install system packages from external file
COPY config/system-packages.list /tmp/system-packages.list
RUN apt-get update && apt-get upgrade -y && \
    xargs -a /tmp/system-packages.list apt-get install -y && \
    # Install ROS package
    apt-get install -y \
        ros-${ROS_DISTRO}-foxglove-bridge \
        ros-${ROS_DISTRO}-ros-gz \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        ros-${ROS_DISTRO}-demo-nodes-cpp \
        ros-${ROS_DISTRO}-demo-nodes-py && \
    rm -rf /var/lib/apt/lists/* /tmp/system-packages.list

# Create user
RUN groupadd --gid ${DEV_GROUP_ID} ${DEV_USER} && \
    useradd --uid ${DEV_USER_ID} --gid ${DEV_GROUP_ID} -m ${DEV_USER} && \
    echo "${DEV_USER} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${DEV_USER} && \
    chmod 0440 /etc/sudoers.d/${DEV_USER}

# Create workspace structure and set ownership
RUN mkdir -p /home/${DEV_USER}/ros_ws/{src/thirdparty_packages,src/host_packages,bags,maps,results,scripts,log,config} && \
    mkdir -p /home/${DEV_USER}/workspace_host && \
    chown -R ${DEV_USER}:${DEV_USER} /home/${DEV_USER}

# Switch to user and copy files
USER ${DEV_USER}
WORKDIR /home/${DEV_USER}/ros_ws

# Copy configuration, scripts, and make executable
COPY --chown=${DEV_USER}:${DEV_USER} config/ /home/${DEV_USER}/ros_ws/config/
COPY --chown=${DEV_USER}:${DEV_USER} scripts/ /home/${DEV_USER}/ros_ws/scripts/
COPY --chown=${DEV_USER}:${DEV_USER} packages.repos /home/${DEV_USER}/ros_ws/

RUN find /home/${DEV_USER}/ros_ws/scripts -name "*.sh" -exec chmod +x {} \;

ENV DEBIAN_FRONTEND=dialog