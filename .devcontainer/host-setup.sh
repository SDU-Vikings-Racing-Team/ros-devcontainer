#!/bin/bash

# HOST ENVIRONMENT SETUP
# Called by devcontainer.json initializeCommand
# Detects environment (CI vs local) and generates docker-compose.override.yml

set -e

SCRIPT_DIR="$(dirname "${BASH_SOURCE[0]}")"
source "${SCRIPT_DIR}/scripts/core/config.sh"
source "${SCRIPT_DIR}/scripts/core/logger.sh"

# ============================================================================
# ENVIRONMENT DETECTION
# ============================================================================

is_ci_environment() {
    [ -n "$CI" ]
}

has_x11_support() {
    [ -n "$DISPLAY" ]
}

setup_x11_access() {
    if [ -z "$XAUTHORITY" ]; then
        export XAUTHORITY="$HOME/.Xauthority"
    fi
    
    if command -v xhost >/dev/null 2>&1; then
        xhost +local:docker 2>/dev/null || log_warning "Could not configure xhost access"
    fi
    
    log_info "X11 access configured"
}

detect_environment() {
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
    
    # Return format: context:feature1,feature2,...
    echo "${context}:$(IFS=,; echo "${features[*]}")"
}

# ============================================================================
# DOCKER COMPOSE GENERATION
# ============================================================================

generate_base_volumes() {
    cat << EOF
      - \${HOME}/.bash_history:${HOME_DIR}/.bash_history:rw
EOF
}

generate_x11_volumes() {
    cat << EOF
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - \${XAUTHORITY:-\${HOME}/.Xauthority}:${HOME_DIR}/.Xauthority:ro
EOF
}

generate_compose_override() {
    local output_file="$1"
    local include_x11="${2:-false}"
    
    cat > "$output_file" << EOF
services:
  ${CONTAINER_SERVICE}:
    volumes:
$(generate_base_volumes)
EOF

    if [ "$include_x11" = "true" ]; then
        generate_x11_volumes >> "$output_file"
    fi
    
    log_success "Generated docker-compose override: $output_file"
}

# ============================================================================
# HOST PREPARATION
# ============================================================================

prepare_host_environment() {
    log_info "Preparing host environment..."
    
    # Create directories
    ensure_directory "${WORKSPACE_ROOT}/bags"
    touch "$HOME/.bash_history" 2>/dev/null || true
}

configure_environment() {
    local env_info
    env_info=$(detect_environment)
    
    local env_type="${env_info%:*}"
    local features="${env_info#*:}"
    
    log_info "Environment: $env_type | Features: ${features:-none}"
    
    local override_file="${SCRIPT_DIR}/docker-compose.override.yml"
    
    case "$env_type" in
        "ci")
            log_info "CI mode - creating minimal override (no X11)"
            generate_compose_override "$override_file" "false"
            ;;
        "local")
            local include_x11="false"
            if [[ "$features" == *"x11"* ]]; then
                include_x11="true"
                log_info "Including X11 support"
            fi
            generate_compose_override "$override_file" "$include_x11"
            ;;
        *)
            log_warning "Unknown environment: $env_type (defaulting to local)"
            generate_compose_override "$override_file" "false"
            ;;
    esac
}

update_gitignore() {
    local gitignore_entry=".devcontainer/docker-compose.override.yml"
    
    if [ -f .gitignore ] && ! grep -q "^${gitignore_entry}$" .gitignore; then
        echo "$gitignore_entry" >> .gitignore
        log_info "Updated .gitignore"
    fi
}

# ============================================================================
# MAIN
# ============================================================================

main() {
    prepare_host_environment
    configure_environment
    update_gitignore
    log_success "Host preparation complete"
}

main "$@"