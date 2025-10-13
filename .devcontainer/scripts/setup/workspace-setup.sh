#!/bin/bash

set -e

# Source paths
source "$(dirname "${BASH_SOURCE[0]}")/../core/paths.sh"
source "${PATH_CORE}/config-loader.sh"
source "${PATH_CORE}/logger.sh"
source "${PATH_CORE}/path-manager.sh"
source "${PATH_STRATEGIES}/package-strategy.sh"

# Setup workspace directory structure
setup_workspace_structure() {
    log_info "Setting up workspace structure..."
    ensure_workspace_structure
}

# Link host packages to workspace
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

# Import third-party packages
import_thirdparty_packages() {
    install_package_dependencies "vcs" "$PACKAGES_REPOS" "$WS_THIRDPARTY"
}

# Main workspace setup
setup_workspace() {
    setup_workspace_structure
    link_host_packages
    import_thirdparty_packages
}

# Execute if called directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    setup_workspace
fi