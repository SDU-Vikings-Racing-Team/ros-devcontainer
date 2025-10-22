#!/bin/bash

# WORKSPACE AND ENVIRONMENT SETUP
# Creates workspace structure, links packages, configures shell

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/core/config.sh"
source "${SCRIPT_DIR}/core/logger.sh"

# ============================================================================
# ROS ENVIRONMENT
# ============================================================================

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

# ============================================================================
# WORKSPACE STRUCTURE
# ============================================================================

setup_workspace_structure() {
    log_info "Setting up workspace structure..."
    ensure_workspace_structure
}

# ============================================================================
# PACKAGE LINKING
# ============================================================================

link_host_packages() {
    if [ ! -d "${HOST_MOUNT}/src" ]; then
        log_info "No host source directory to link"
        return 0
    fi
    
    if ! has_content "${HOST_MOUNT}/src"; then
        log_info "Host source directory is empty"
        return 0
    fi
    
    log_info "Linking host packages..."
    
    ensure_directory "$WS_HOST_PACKAGES"
    
    for package in "${HOST_MOUNT}/src"/*; do
        if [ -d "$package" ]; then
            local package_name
            package_name=$(basename "$package")
            local target="${WS_HOST_PACKAGES}/${package_name}"
            
            if [ ! -e "$target" ]; then
                ln -s "$package" "$target"
                log_info "Linked: $package_name"
            else
                log_info "Already linked: $package_name"
            fi
        fi
    done
    
    log_success "Host packages linked"
}

# ============================================================================
# THIRD-PARTY PACKAGES (VCS)
# ============================================================================

import_vcs_packages() {
    local repos_file="$PACKAGES_REPOS"
    
    if [ ! -f "$repos_file" ]; then
        log_info "No packages.repos file found"
        return 1
    fi
    
    # Check if file has actual repository entries
    if ! grep -qvE '^\s*(#|$)' "$repos_file" || ! grep -q 'type:' "$repos_file"; then
        log_info "packages.repos contains no active repositories"
        return 1
    fi
    
    log_info "Importing packages from VCS..."
    
    ensure_directory "$WS_THIRDPARTY"
    cd "$WS_THIRDPARTY"
    
    # Install vcs tool if needed
    if ! command -v vcs &> /dev/null; then
        log_info "Installing python3-vcstool..."
        source "${SCRIPT_DIR}/core/cache.sh"
        update_apt_cache
        sudo apt-get install -y python3-vcstool
    fi
    
    if vcs import < "$repos_file"; then
        log_success "VCS packages imported"
        return 0
    else
        log_error "Failed to import VCS packages"
        return 1
    fi
}

# ============================================================================
# SHELL ENVIRONMENT
# ============================================================================

configure_shell_environment() {
    log_info "Configuring shell environment..."
    
    if grep -q "# ROS2 Environment Setup" ~/.bashrc; then
        log_info "Shell environment already configured"
        return 0
    fi
    
    cat >> ~/.bashrc << 'EOF'

# ROS2 Environment Setup
# Source config first to define all variables
if [ -f /home/rosdev/ros_ws/config/defaults.env ]; then
    set -a
    source /home/rosdev/ros_ws/config/defaults.env
    set +a
fi

# Now variables are defined, use them
if [ -f ${ROS_ROOT}/setup.bash ]; then
    source ${ROS_ROOT}/setup.bash
fi

if [ -f ${WORKSPACE_ROOT}/install/setup.bash ]; then
    source ${WORKSPACE_ROOT}/install/setup.bash
fi

# ROS Aliases
alias cb='cd ${WORKSPACE_ROOT} && colcon build ${COLCON_BUILD_ARGS}'
alias cbp='cd ${WORKSPACE_ROOT} && colcon build ${COLCON_BUILD_ARGS} --packages-select'
alias cbu='cd ${WORKSPACE_ROOT} && colcon build ${COLCON_BUILD_ARGS} --packages-up-to'
alias ct='cd ${WORKSPACE_ROOT} && colcon test'
alias ctp='cd ${WORKSPACE_ROOT} && colcon test --packages-select'
alias ctr='cd ${WORKSPACE_ROOT} && colcon test-result --verbose'
alias source_ws='source ${WORKSPACE_ROOT}/install/setup.bash'
alias clean_ws='cd ${WORKSPACE_ROOT} && rm -rf build install log'
alias rosdep_install='cd ${WORKSPACE_ROOT} && rosdep install --from-paths src --ignore-src -r -y'

# Shell Aliases
alias ll='ls -alF --color=auto'
alias la='ls -A --color=auto'
alias l='ls -CF --color=auto'

# Auto-cd to workspace
cd ${WORKSPACE_ROOT}
EOF
    
    log_success "Shell environment configured"
}

# ============================================================================
# MAIN SETUP
# ============================================================================

main() {
    setup_workspace_structure
    link_host_packages
    import_vcs_packages
    configure_shell_environment
}

# Execute if called directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi