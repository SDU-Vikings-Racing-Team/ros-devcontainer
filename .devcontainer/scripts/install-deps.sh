#!/bin/bash

# Install ROS dependencies

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/utils.sh"

install_dependencies() {
    local target_path="$1"
    local description="$2"
    
    if has_content "$target_path"; then
        log_info "Installing dependencies for ${description}..."
        
        # Update package lists if needed
        if [ ! -f /tmp/.apt_updated ]; then
            log_info "Updating apt package lists..."
            sudo apt-get update && touch /tmp/.apt_updated
        fi
        
        # Update and use rosdep
        update_rosdep
        
        cd "${WS_ROOT}"
        if rosdep install --from-paths "$target_path" --ignore-src -r -y; then
            log_success "Dependencies installed for ${description}"
            return 0
        else
            log_warning "Some dependencies could not be installed for ${description}"
            return 0  # Continue anyway
        fi
    else
        log_info "No packages found in ${target_path}"
        return 1
    fi
}

source_ros

# Install dependencies for different package groups
any_installed=false

if install_dependencies "${WS_THIRDPARTY}" "third-party packages"; then
    any_installed=true
fi

if install_dependencies "${WS_HOST_PACKAGES}" "host packages"; then
    any_installed=true
fi

if [ "$any_installed" = false ]; then
    log_info "No dependencies to install"
fi