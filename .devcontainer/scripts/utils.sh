#!/bin/bash

# Utility functions

# Load config
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/config.sh"

# Logging functions
log_info() {
    echo "[INFO] $*"
}

log_error() {
    echo "[ERROR] $*" >&2
}

log_success() {
    echo "[SUCCESS] $*"
}

log_warning() {
    echo "[WARNING] $*"
}

# Source ROS environment
source_ros() {
    if [ -f "${ROS_ROOT}/setup.bash" ]; then
        source "${ROS_ROOT}/setup.bash"
        log_info "Sourced ROS ${ROS_DISTRO}"
        return 0
    else
        log_error "ROS setup.bash not found at ${ROS_ROOT}"
        return 1
    fi
}

# Source workspace if it exists
source_workspace() {
    if [ -f "${WS_ROOT}/install/setup.bash" ]; then
        source "${WS_ROOT}/install/setup.bash"
        log_info "Sourced workspace"
        return 0
    fi
    return 1
}

# Check if directory has content
has_content() {
    [ -d "$1" ] && [ "$(ls -A "$1" 2>/dev/null)" ]
}

# Update rosdep (with caching to avoid redundant calls)
update_rosdep() {
    local cache_file="/tmp/.rosdep_updated"
    local max_age=3600  # 1 hour in seconds
    
    if [ -f "$cache_file" ]; then
        local age=$(($(date +%s) - $(stat -c %Y "$cache_file")))
        if [ $age -lt $max_age ]; then
            log_info "Rosdep recently updated, skipping..."
            return 0
        fi
    fi
    
    log_info "Updating rosdep..."
    if rosdep update; then
        touch "$cache_file"
        return 0
    else
        log_error "Failed to update rosdep"
        return 1
    fi
}

# Ensure directory exists
ensure_dir() {
    local dir="$1"
    if [ ! -d "$dir" ]; then
        if mkdir -p "$dir" 2>/dev/null; then
            log_info "Created directory: $dir"
        else
            log_warning "Could not create $dir (may already exist or be mounted)"
        fi
    fi
}