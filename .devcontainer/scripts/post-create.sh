#!/bin/bash

# Orchestrator script - runs after container creation

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/utils.sh"

log_info "Starting post-create setup..."

# Fix permissions for mounted volumes
log_info "Fixing permissions..."
sudo chown -R rosdev:rosdev /home/rosdev/.ros 2>/dev/null || true
sudo chown -R rosdev:rosdev ${WS_ROOT}/build 2>/dev/null || true
sudo chown -R rosdev:rosdev ${WS_ROOT}/install 2>/dev/null || true

# Execute setup scripts in order
"${SCRIPT_DIR}/setup-workspace.sh"
"${SCRIPT_DIR}/setup-environment.sh"

# Only proceed with deps and build if there are packages
if has_content "${WS_SRC}"; then
    "${SCRIPT_DIR}/install-deps.sh"
    "${SCRIPT_DIR}/build.sh"
fi

# Summary
echo ""
echo "============================="
echo "       Setup Complete!       "
echo "============================="
echo ""

if has_content "${WS_HOST_PACKAGES}"; then
    echo "Host packages:"
    for link in "${WS_HOST_PACKAGES}"/*; do
        if [ -L "$link" ]; then
            echo "   - $(basename "$link")"
        fi
    done
    echo ""
fi

if has_content "${WS_THIRDPARTY}"; then
    echo "Third-party packages:"
    for pkg in "${WS_THIRDPARTY}"/*; do
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

if [ -f "${WS_ROOT}/install/setup.bash" ]; then
    echo "Workspace built and ready to use!"
else
    echo "No packages built yet. Add packages to src/ and run 'cb'"
fi

log_success "Development environment ready!"