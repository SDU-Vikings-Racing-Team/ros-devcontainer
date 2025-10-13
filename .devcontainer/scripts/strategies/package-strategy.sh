#!/bin/bash

# Source paths
source "$(dirname "${BASH_SOURCE[0]}")/../core/paths.sh"
source "${PATH_CORE}/config-loader.sh"
source "${PATH_CORE}/logger.sh"
source "${PATH_CORE}/path-manager.sh"
source "${PATH_CORE}/cache-manager.sh"
source "${PATH_COMMANDS}/command-factory.sh"

# Base strategy "interface"
install_package_dependencies() {
    local strategy="$1"
    local package_path="$2"
    local description="$3"
    
    case "$strategy" in
        "host")
            install_host_package_dependencies "$package_path" "$description"
            ;;
        "thirdparty")
            install_thirdparty_package_dependencies "$package_path" "$description"
            ;;
        "vcs")
            install_vcs_packages "$package_path" "$description"
            ;;
        *)
            log_error "Unknown package strategy: $strategy"
            return 1
            ;;
    esac
}

# Host package strategy
install_host_package_dependencies() {
    local package_path="$1"
    local description="${2:-host packages}"
    
    if has_content "$package_path"; then
        log_info "Installing dependencies for $description..."
        
        update_apt_cache
        update_rosdep
        
        local rosdep_cmd
        rosdep_cmd=$(build_rosdep_command "$package_path")
        
        if execute_command "rosdep install for $description" "$rosdep_cmd"; then
            return 0
        else
            log_warning "Some dependencies could not be installed for $description"
            return 0  # Continue anyway
        fi
    else
        log_info "No packages found in $package_path"
        return 1
    fi
}

# Third-party package strategy  
install_thirdparty_package_dependencies() {
    install_host_package_dependencies "$@"  # Same logic for now
}

# VCS package strategy
install_vcs_packages() {
    local repos_file="${1:-$PACKAGES_REPOS}"
    local target_dir="${2:-$WS_THIRDPARTY}"
    
    if [ ! -f "$repos_file" ]; then
        log_info "No packages.repos file found"
        return 1
    fi
    
    # Check if file has repository entries (not just comments)
    if ! grep -qvE '^\s*(#|$)' "$repos_file" || ! grep -q 'type:' "$repos_file"; then
        log_info "packages.repos contains no active repositories"
        return 1
    fi
    
    log_info "Importing packages from $repos_file..."
    
    ensure_directory "$target_dir"
    cd "$target_dir"
    
    # Install vcs if needed
    if ! command -v vcs &> /dev/null; then
        log_info "Installing python3-vcstool..."
        update_apt_cache
        sudo apt-get install -y python3-vcstool
    fi
    
    if vcs import < "$repos_file"; then
        log_success "Packages imported successfully"
        return 0
    else
        log_error "Failed to import packages"
        return 1
    fi
}