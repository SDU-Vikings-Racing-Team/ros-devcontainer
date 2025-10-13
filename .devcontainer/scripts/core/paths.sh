#!/bin/bash

# PATH MANAGEMENT

is_inside_container() {
    # Checks to see if we are inside the devcontainer
    [[ "$HOME" == "/home/rosdev" ]] || \
    [[ "$(pwd)" == "/home/rosdev/ros_ws"* ]] || \
    [[ -f "/.dockerenv" ]] || \
    [[ "$USER" == "rosdev" ]]
}

# Find project root by looking for .devcontainer directory
find_project_root() {
    local current_dir
    current_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    
    # Traverse up until we find .devcontainer directory
    while [[ "$current_dir" != "/" ]]; do
        if [[ -d "$current_dir/.devcontainer" ]]; then
            echo "$current_dir"
            return 0
        fi
        current_dir="$(dirname "$current_dir")"
    done
    
    # Fallback: extract path before .devcontainer
    current_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    if [[ "$current_dir" == *"/.devcontainer/"* ]]; then
        echo "${current_dir%%/.devcontainer/*}"
        return 0
    fi
    
    echo "ERROR: Could not find project root" >&2
    return 1
}

# This is only set once, never overwritten
if [[ -z "$_PATHS_INITIALIZED" ]]; then
    export _PATHS_INITIALIZED="true"
    
    if is_inside_container; then
        # Inside container: scripts are at /home/rosdev/ros_ws/scripts/
        export WORKSPACE_ROOT="/home/rosdev/ros_ws"
        export PATH_SCRIPTS="${WORKSPACE_ROOT}/scripts"
        export PATH_CONFIG="${WORKSPACE_ROOT}/config" 
        export WORKSPACE_SRC="${WORKSPACE_ROOT}/src"
        export _RUNNING_ON_HOST="false"
    else
        # On host: scripts are in .devcontainer/scripts/
        export WORKSPACE_ROOT="$(find_project_root)"
        export PATH_SCRIPTS="${WORKSPACE_ROOT}/.devcontainer/scripts"
        export PATH_CONFIG="${WORKSPACE_ROOT}/.devcontainer/config"
        export WORKSPACE_SRC="/home/rosdev/ros_ws/src" # Container path reference
        export _RUNNING_ON_HOST="true"
    fi
    
    # These paths should never be overwritten!
    export PATH_CORE="${PATH_SCRIPTS}/core"
    export PATH_SETUP="${PATH_SCRIPTS}/setup" 
    export PATH_ENVIRONMENT="${PATH_SCRIPTS}/environment"
    export PATH_GENERATORS="${PATH_SCRIPTS}/generators"
    export PATH_COMMANDS="${PATH_SCRIPTS}/commands"
    export PATH_STRATEGIES="${PATH_SCRIPTS}/strategies"
    
    # Workspace src paths
    export WS_HOST_PACKAGES="${WORKSPACE_SRC}/host_packages"
    export WS_THIRDPARTY="${WORKSPACE_SRC}/thirdparty_packages"
    
    # Config and cache paths  
    export CONFIG_FILE="${PATH_CONFIG}/defaults.env"
    export PACKAGES_REPOS="${WORKSPACE_ROOT}/packages.repos"
    
    export CACHE_DIR="${HOME}/.cache/ros_devcontainer"
    export APT_CACHE_FILE="${CACHE_DIR}/.apt_updated"
    export ROSDEP_CACHE_FILE="${CACHE_DIR}/.rosdep_updated"
    
    # Ensure cache directory exists
    mkdir -p "$CACHE_DIR" 2>/dev/null || true
    
    # Home and host mount paths
    export HOME_DIR="/home/${DEV_USER:-rosdev}"
    export HOST_MOUNT="${HOME_DIR}/workspace_host"
fi