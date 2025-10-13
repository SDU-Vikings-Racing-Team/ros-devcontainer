#!/bin/bash

# ROS ENVIRONMENT

source "$(dirname "${BASH_SOURCE[0]}")/../core/config-loader.sh"
source "$(dirname "${BASH_SOURCE[0]}")/../core/logger.sh"

# Source ROS environment
source_ros() {
    local setup_file="${ROS_ROOT}/setup.bash"
    
    if [ -f "$setup_file" ]; then
        source "$setup_file"
        log_info "Sourced ROS ${ROS_DISTRO}"
        return 0
    else
        log_error "ROS setup.bash not found at ${setup_file}"
        return 1
    fi
}

# Source workspace if it exists
source_workspace() {
    local setup_file="${WORKSPACE_ROOT}/install/setup.bash"
    
    if [ -f "$setup_file" ]; then
        source "$setup_file"
        log_info "Sourced workspace"
        return 0
    else
        log_debug "Workspace not built yet"
        return 1
    fi
}

# Setup complete ROS environment
setup_ros_environment() {
    source_ros
    source_workspace || true # OK if workspace does not exist yet
}