#!/bin/bash

# Source paths
source "$(dirname "${BASH_SOURCE[0]}")/paths.sh"

load_config() {
    if [ -f "$CONFIG_FILE" ]; then
        # Store host paths that should not be overwritten
        local host_workspace_root="$WORKSPACE_ROOT"
        local host_path_scripts="$PATH_SCRIPTS"
        local host_path_config="$PATH_CONFIG"
        local host_path_core="$PATH_CORE"
        
        set -a
        source "$CONFIG_FILE"
        set +a
        
        # Restore host paths if we are running on host
        if [ "$_RUNNING_ON_HOST" = "true" ]; then
            export WORKSPACE_ROOT="$host_workspace_root"
            export PATH_SCRIPTS="$host_path_scripts"
            export PATH_CONFIG="$host_path_config"
            export PATH_CORE="$host_path_core"
        fi
        
        # Paths that are OK to override
        export ROS_ROOT="/opt/ros/${ROS_DISTRO}"
        export COLCON_BUILD_ARGS="${COLCON_BASE_ARGS} --cmake-args ${CMAKE_BASE_ARGS}"
        
        return 0
    else
        echo "ERROR: Configuration file not found: $CONFIG_FILE" >&2
        return 1
    fi
}

load_config