#!/bin/bash

# Single source of truth for all config

# ROS config
export ROS_DISTRO="${ROS_DISTRO:-humble}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
export ROS_ROOT="/opt/ros/${ROS_DISTRO}"
export ROS_PYTHON_VERSION=3

# Logging config
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export RCUTILS_LOGGING_USE_STDOUT=1

# Workspace config
export WS_ROOT="/home/rosdev/ros_ws"
export WS_SRC="${WS_ROOT}/src"
export WS_HOST_PACKAGES="${WS_SRC}/host_packages"
export WS_THIRDPARTY="${WS_SRC}/thirdparty_packages"
export HOST_MOUNT="/home/rosdev/workspace_host"

# Build config
export BUILD_TYPE="Release"
export COLCON_BUILD_ARGS="--symlink-install --cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE}"

# Directory structure
export WS_DIRS=(
    "${WS_SRC}"
    "${WS_HOST_PACKAGES}"
    "${WS_THIRDPARTY}"
    "${WS_ROOT}/bags"
    "${WS_ROOT}/maps"
    "${WS_ROOT}/results"
)