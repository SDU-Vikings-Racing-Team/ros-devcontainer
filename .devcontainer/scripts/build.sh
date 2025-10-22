#!/bin/bash

# BUILD SCRIPT

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/core/config.sh"
source "${SCRIPT_DIR}/core/logger.sh"

build_workspace() {
    local additional_args="$*"
    
    if ! has_content "$WORKSPACE_SRC"; then
        log_info "No packages to build"
        return 1
    fi
    
    log_info "Building workspace..."
    
    # Source ROS environment
    local ros_setup="${ROS_ROOT}/setup.bash"
    if [ -f "$ros_setup" ]; then
        source "$ros_setup"
    else
        log_error "ROS setup not found"
        return 1
    fi
    
    # Source workspace if it exists
    local ws_setup="${WORKSPACE_ROOT}/install/setup.bash"
    if [ -f "$ws_setup" ]; then
        source "$ws_setup"
    fi
    
    cd "${WORKSPACE_ROOT}"
    
    if colcon build ${COLCON_BUILD_ARGS} ${additional_args}; then
        log_success "Workspace build completed"
        return 0
    else
        log_error "Workspace build failed"
        return 1
    fi
}

# Execute if called directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    build_workspace "$@"
fi