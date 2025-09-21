#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/utils.sh"

REPO_URL="https://github.com/rsasaki0109/lidarslam_ros2"
REPO_NAME="lidarslam_ros2"
TARGET_DIR="${WS_THIRDPARTY}/${REPO_NAME}"

install_lidarslam() {
    log_info "Installing lidarslam_ros2..."
    
    ensure_dir "${WS_THIRDPARTY}"
    
    # Check if already installed
    if [ -d "${TARGET_DIR}" ]; then
        log_warning "lidarslam_ros2 already exists at ${TARGET_DIR}"
        read -p "Remove existing installation and reinstall? [y/N]: " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            log_info "Removing existing installation..."
            rm -rf "${TARGET_DIR}"
        else
            log_info "Keeping existing installation"
            return 0
        fi
    fi
    
    # Clone the repository
    log_info "Cloning ${REPO_URL}..."
    cd "${WS_THIRDPARTY}"
    
    if git clone --recursive "${REPO_URL}"; then
        log_success "Successfully cloned lidarslam_ros2"
    else
        log_error "Failed to clone lidarslam_ros2"
        return 1
    fi
    
    log_info "Installing dependencies for lidarslam_ros2..."
    cd "${WS_ROOT}"
    
    source_ros
    
    update_rosdep
    
    # Install deps
    if rosdep install --from-paths src --ignore-src -r -y; then
        log_success "Dependencies installed successfully"
    else
        log_warning "Some dependencies could not be installed, but continuing..."
    fi
    
    log_success "lidarslam_ros2 installation complete!"
    echo ""
    echo "Next steps:"
    echo "  1. Run 'cb' to build the workspace"
    echo "  2. Source the workspace: 'source_ws'"
    echo ""
}

log_info "Starting lidarslam_ros2 installation..."
install_lidarslam