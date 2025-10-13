#!/bin/bash

# Source paths
source "$(dirname "${BASH_SOURCE[0]}")/../core/paths.sh"
source "${PATH_CORE}/logger.sh"
source "${PATH_CORE}/config-loader.sh"

# Detect if running in CI environment
is_ci_environment() {
    [ -n "$CI" ] || [ -n "$GITHUB_ACTIONS" ]
}

# Check X11 availability
has_x11_support() {
    [ -n "$DISPLAY" ]
}

# Setup X11 access permissions
setup_x11_access() {
    if [ -z "$XAUTHORITY" ]; then
        export XAUTHORITY="$HOME/.Xauthority"
    fi
    
    if command -v xhost >/dev/null 2>&1; then
        xhost +local:docker 2>/dev/null || log_warning "Could not configure xhost access"
    fi
    
    log_info "X11 access configured"
}

detect_environment_context() {
    local context="local"
    local features=()
    
    if is_ci_environment; then
        context="ci"
    else
        if has_x11_support; then
            features+=("x11")
            setup_x11_access
        fi
    fi
    
    # Return format: context: feature1, feature2, ...
    echo "${context}:$(IFS=,; echo "${features[*]}")"
}