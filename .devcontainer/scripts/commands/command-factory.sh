#!/bin/bash

# Building commands

source "$(dirname "${BASH_SOURCE[0]}")/../core/config-loader.sh"

# Build colcon command with additional arguments
build_colcon_command() {
    local additional_args="$*"
    echo "cd ${WORKSPACE_ROOT} && colcon build ${COLCON_BUILD_ARGS} ${additional_args}"
}

# Build rosdep install command
build_rosdep_command() {
    local target_path="$1"
    echo "cd ${WORKSPACE_ROOT} && rosdep install --from-paths \"$target_path\" --ignore-src -r -y"
}

# Build colcon test command
build_test_command() {
    local additional_args="$*"
    echo "cd ${WORKSPACE_ROOT} && colcon test ${additional_args}"
}

# Execute command with logging
execute_command() {
    local description="$1"
    local command="$2"
    
    log_info "Executing: $description"
    log_debug "Command: $command"
    
    if eval "$command"; then
        log_success "$description completed"
        return 0
    else
        log_error "$description failed"
        return 1
    fi
}