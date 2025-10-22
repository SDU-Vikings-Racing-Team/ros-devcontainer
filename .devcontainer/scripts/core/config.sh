#!/bin/bash

# CONFIGURATION AND PATH MANAGEMENT
# Detects host vs container, loads config, provides directory utilities

# ============================================================================
# PATH DETECTION
# ============================================================================

is_inside_container() {
    [[ "$HOME" == "/home/rosdev" ]] || \
    [[ "$(pwd)" == "/home/rosdev/ros_ws"* ]] || \
    [[ -f "/.dockerenv" ]] || \
    [[ "$USER" == "rosdev" ]]
}

find_project_root() {
    local current_dir
    current_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    
    # Traverse up to find .devcontainer
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

# Initialize paths once
if [[ -z "$_PATHS_INITIALIZED" ]]; then
    export _PATHS_INITIALIZED="true"
    
    if is_inside_container; then
        export WORKSPACE_ROOT="/home/rosdev/ros_ws"
        export SCRIPTS_DIR="${WORKSPACE_ROOT}/scripts"
        export CONFIG_DIR="${WORKSPACE_ROOT}/config"
        export _RUNNING_ON_HOST="false"
    else
        export WORKSPACE_ROOT="$(find_project_root)"
        export SCRIPTS_DIR="${WORKSPACE_ROOT}/.devcontainer/scripts"
        export CONFIG_DIR="${WORKSPACE_ROOT}/.devcontainer/config"
        export _RUNNING_ON_HOST="true"
    fi
    
    export WORKSPACE_SRC="${WORKSPACE_ROOT}/src"
    export WS_HOST_PACKAGES="${WORKSPACE_SRC}/host_packages"
    export WS_THIRDPARTY="${WORKSPACE_SRC}/thirdparty_packages"
    
    export CACHE_DIR="${HOME}/.cache/ros_devcontainer"
    mkdir -p "$CACHE_DIR" 2>/dev/null || true
fi

# ============================================================================
# CONFIGURATION LOADING
# ============================================================================

load_config() {
    local config_file="${CONFIG_DIR}/defaults.env"
    
    if [ ! -f "$config_file" ]; then
        echo "ERROR: Configuration file not found: $config_file" >&2
        return 1
    fi
    
    # Store critical paths
    local saved_workspace_root="$WORKSPACE_ROOT"
    local saved_scripts_dir="$SCRIPTS_DIR"
    local saved_config_dir="$CONFIG_DIR"
    
    # Load environment variables
    set -a
    source "$config_file"
    set +a
    
    # Restore paths if on host (don't let config override them)
    if [ "$_RUNNING_ON_HOST" = "true" ]; then
        export WORKSPACE_ROOT="$saved_workspace_root"
        export SCRIPTS_DIR="$saved_scripts_dir"
        export CONFIG_DIR="$saved_config_dir"
    fi
    
    # Derived variables
    export ROS_ROOT="/opt/ros/${ROS_DISTRO}"
    export COLCON_BUILD_ARGS="${COLCON_BASE_ARGS} --cmake-args ${CMAKE_BASE_ARGS}"
    export HOST_MOUNT="${HOME_DIR}/workspace_host"
    export PACKAGES_REPOS="${WORKSPACE_ROOT}/packages.repos"
    
    # Cache files
    export APT_CACHE_FILE="${CACHE_DIR}/.apt_updated"
    export ROSDEP_CACHE_FILE="${CACHE_DIR}/.rosdep_updated"
    
    return 0
}

# ============================================================================
# DIRECTORY UTILITIES
# ============================================================================

has_content() {
    [ -d "$1" ] && [ "$(ls -A "$1" 2>/dev/null)" ]
}

ensure_directory() {
    local dir="$1"
    local description="${2:-$(basename "$dir")}"
    
    if [ ! -d "$dir" ]; then
        if mkdir -p "$dir" 2>/dev/null; then
            # Only log if logger is available
            if command -v log_info &>/dev/null; then
                log_info "Created directory: $description"
            fi
            return 0
        else
            if command -v log_warning &>/dev/null; then
                log_warning "Could not create $description"
            fi
            return 1
        fi
    fi
    return 0
}

ensure_workspace_structure() {
    local workspace_root="${1:-$WORKSPACE_ROOT}"
    
    if command -v log_info &>/dev/null; then
        log_info "Ensuring workspace structure..."
    fi
    
    IFS=',' read -ra DIRS <<< "$WORKSPACE_SUBDIRS"
    for subdir in "${DIRS[@]}"; do
        ensure_directory "${workspace_root}/${subdir}" "$subdir"
    done
    
    if command -v log_success &>/dev/null; then
        log_success "Workspace structure ready"
    fi
}

# Auto-load configuration
load_config