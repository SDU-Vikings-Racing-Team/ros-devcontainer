#!/bin/bash

# Entry point after container creation  

set -e

# Source paths first
source "$(dirname "${BASH_SOURCE[0]}")/core/paths.sh"

# Source utils using paths
source "${PATH_CORE}/config-loader.sh"
source "${PATH_CORE}/logger.sh"
source "${PATH_CORE}/path-manager.sh"

log_info "Starting post-create setup..."

# Fix permissions for mounted volumes
fix_permissions() {
    log_info "Fixing permissions..."
    sudo chown -R "${DEV_USER}:${DEV_USER}" "${HOME_DIR}/.ros" 2>/dev/null || true
    sudo chown -R "${DEV_USER}:${DEV_USER}" "${WORKSPACE_ROOT}/build" 2>/dev/null || true
    sudo chown -R "${DEV_USER}:${DEV_USER}" "${WORKSPACE_ROOT}/install" 2>/dev/null || true
}

# Execute setup scripts in dependency order
execute_setup_sequence() {
    "${PATH_SETUP}/workspace-setup.sh"
    "${PATH_ENVIRONMENT}/environment-setup.sh"
    
    # Only proceed with deps and build if there are packages
    if has_content "$WORKSPACE_SRC"; then
        "${PATH_SCRIPTS}/install-deps.sh"
        "${PATH_SCRIPTS}/build.sh"
    fi
}

# Display setup summary
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

main() {
    fix_permissions
    execute_setup_sequence
    display_summary
    log_success "Development environment ready!"
}

main "$@"