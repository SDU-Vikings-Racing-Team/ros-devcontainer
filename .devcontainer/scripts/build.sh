#!/bin/bash

# BUILD SCRIPT

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/core/config-loader.sh"
source "${SCRIPT_DIR}/core/logger.sh"
source "${SCRIPT_DIR}/core/path-manager.sh"
source "${SCRIPT_DIR}/environment/ros-environment.sh"
source "${SCRIPT_DIR}/commands/command-factory.sh"

# Build workspace
build_workspace() {
    local additional_args="$*"
    
    if ! has_content "$WORKSPACE_SRC"; then
        log_info "No packages to build"
        return 1
    fi
    
    log_info "Building workspace..."
    
    setup_ros_environment
    
    local build_cmd
    build_cmd=$(build_colcon_command "$additional_args")
    
    if execute_command "workspace build" "$build_cmd"; then
        return 0
    else
        return 1
    fi
}

# Execute if called directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    build_workspace "$@"
fi