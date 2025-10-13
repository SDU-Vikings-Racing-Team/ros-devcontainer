#!/bin/bash

# DEPENDENCY INSTALLATION

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/core/config-loader.sh"
source "${SCRIPT_DIR}/core/logger.sh"
source "${SCRIPT_DIR}/environment/ros-environment.sh"
source "${SCRIPT_DIR}/strategies/package-strategy.sh"

# Install all dependencies
install_all_dependencies() {
    source_ros
    
    local any_installed=false
    
    # Install dependencies for third-party packages
    if install_package_dependencies "thirdparty" "$WS_THIRDPARTY" "third-party packages"; then
        any_installed=true
    fi
    
    # Install dependencies for host packages  
    if install_package_dependencies "host" "$WS_HOST_PACKAGES" "host packages"; then
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