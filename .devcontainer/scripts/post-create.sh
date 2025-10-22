#!/bin/bash

# POST-CREATE ENTRY POINT
# Runs after container creation to set up workspace

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/core/config.sh"
source "${SCRIPT_DIR}/core/logger.sh"

log_info "Starting post-create setup..."

# ============================================================================
# FIX PERMISSIONS
# ============================================================================

fix_permissions() {
    log_info "Fixing permissions..."
    sudo chown -R "${DEV_USER}:${DEV_USER}" "${HOME_DIR}/.ros" 2>/dev/null || true
    sudo chown -R "${DEV_USER}:${DEV_USER}" "${WORKSPACE_ROOT}/build" 2>/dev/null || true
    sudo chown -R "${DEV_USER}:${DEV_USER}" "${WORKSPACE_ROOT}/install" 2>/dev/null || true
}

# ============================================================================
# CONFIGURE GIT SAFE DIRECTORIES
# ============================================================================

configure_git_safe_directories() {
    log_info "Configuring Git safe directories..."
    
    # Trust all repositories under workspace_host
    if [ -d "${HOST_MOUNT}" ]; then
        git config --global --add safe.directory "${HOST_MOUNT}/*"
        log_success "Git configured to trust workspace_host repositories"
    fi
    
    # Also trust the workspace root
    if [ -d "${WORKSPACE_ROOT}/.git" ]; then
        git config --global --add safe.directory "${WORKSPACE_ROOT}"
    fi
}

# ============================================================================
# SETUP SEQUENCE
# ============================================================================

execute_setup_sequence() {
    "${SCRIPT_DIR}/setup.sh"
    
    # Only proceed with deps and build if there are packages
    if has_content "$WORKSPACE_SRC"; then
        "${SCRIPT_DIR}/install-deps.sh"
        "${SCRIPT_DIR}/build.sh"
    fi
}

# ============================================================================
# SUMMARY
# ============================================================================

display_summary() {
    echo ""
    echo "============================="
    echo "       Setup Complete!       "
    echo "============================="
    echo ""
    
    if has_content "$WS_HOST_PACKAGES"; then
        echo "Host packages:"
        for link in "$WS_HOST_PACKAGES"/*; do
            if [ -L "$link" ]; then
                echo "   - $(basename "$link")"
            fi
        done
        echo ""
    fi
    
    if has_content "$WS_THIRDPARTY"; then
        echo "Third-party packages:"
        for pkg in "$WS_THIRDPARTY"/*; do
            if [ -d "$pkg" ]; then
                echo "   - $(basename "$pkg")"
            fi
        done
        echo ""
    fi
    
    echo "Useful commands:"
    echo "   cb         - Build workspace"
    echo "   cbp <pkg>  - Build specific package"
    echo "   cbu <pkg>  - Build up to package"
    echo "   source_ws  - Source workspace"
    echo "   clean_ws   - Clean build artifacts"
    echo ""
    
    if [ -f "${WORKSPACE_ROOT}/install/setup.bash" ]; then
        echo "Workspace built and ready to use!"
    else
        echo "No packages built yet. Add packages to src/ and run 'cb'"
    fi
}

# ============================================================================
# MAIN
# ============================================================================

main() {
    fix_permissions
    configure_git_safe_directories
    execute_setup_sequence
    display_summary
    log_success "Development environment ready!"
}

main "$@"