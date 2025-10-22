#!/bin/bash

# DEPENDENCY INSTALLATION
# Installs ROS dependencies using rosdep

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/core/config.sh"
source "${SCRIPT_DIR}/core/logger.sh"
source "${SCRIPT_DIR}/core/cache.sh"

# ============================================================================
# ROSDEP INSTALLATION
# ============================================================================

install_rosdep_dependencies() {
    local package_path="$1"
    local description="${2:-packages}"
    
    if ! has_content "$package_path"; then
        log_info "No packages found in $package_path"
        return 1
    fi
    
    log_info "Installing dependencies for $description..."
    
    update_apt_cache
    update_rosdep
    
    cd "${WORKSPACE_ROOT}"
    
    if rosdep install --from-paths "$package_path" --ignore-src -r -y; then
        log_success "Dependencies installed for $description"
        return 0
    else
        log_warning "Some dependencies could not be installed for $description"
        return 0  # Continue anyway
    fi
}

# ============================================================================
# MAIN
# ============================================================================

install_all_dependencies() {
    # Source ROS environment
    local ros_setup="${ROS_ROOT}/setup.bash"
    if [ -f "$ros_setup" ]; then
        source "$ros_setup"
        log_info "Sourced ROS ${ROS_DISTRO}"
    else
        log_error "ROS setup not found"
        return 1
    fi
    
    local any_installed=false
    
    # Install dependencies for host packages
    if install_rosdep_dependencies "$WS_HOST_PACKAGES" "host packages"; then
        any_installed=true
    fi
    
    # Install dependencies for third-party packages
    if install_rosdep_dependencies "$WS_THIRDPARTY" "third-party packages"; then
        any_installed=true
    fi
    
    if [ "$any_installed" = false ]; then
        log_info "No dependencies to install"
    fi
}

# Execute if called directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    install_all_dependencies
fi