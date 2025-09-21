#!/bin/bash

# Build ROS workspace

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/utils.sh"

build_workspace() {
    if has_content "${WS_SRC}"; then
        log_info "Building workspace..."
        cd "${WS_ROOT}"
        
        source_ros
        source_workspace || true  # OK if workspace doesn't exist yet
        
        if eval colcon build ${COLCON_BUILD_ARGS}; then
            log_success "Workspace built successfully"
            return 0
        else
            log_error "Build failed"
            return 1
        fi
    else
        log_info "No packages to build"
        return 1
    fi
}

build_workspace