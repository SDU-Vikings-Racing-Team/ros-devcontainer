#!/bin/bash

set -e

# Source paths first
source "$(dirname "${BASH_SOURCE[0]}")/scripts/core/paths.sh"

# Then source all other deps
source "${PATH_CORE}/config-loader.sh"
source "${PATH_CORE}/logger.sh" 
source "${PATH_CORE}/path-manager.sh"
source "${PATH_ENVIRONMENT}/environment-detector.sh"
source "${PATH_GENERATORS}/docker-compose-generator.sh"

prepare_host_environment() {
    log_info "Preparing host environment for devcontainer..."
    ensure_directory "bags"
    touch "$HOME/.bash_history" 2>/dev/null || true
}

configure_environment() {
    local env_info
    env_info=$(detect_environment_context)
    
    local env_type="${env_info%:*}"
    local features="${env_info#*:}"
    
    log_info "Environment detected: $env_type with features: ${features:-none}"
    
    case "$env_type" in
        "ci")
            log_info "CI environment - using base configuration only"
            rm -f "$(dirname "${BASH_SOURCE[0]}")/docker-compose.override.yml"
            ;;
        "local")
            create_environment_override "$(dirname "${BASH_SOURCE[0]}")/docker-compose.override.yml" "$features"
            ;;
        *)
            log_warning "Unknown environment type: $env_type"
            ;;
    esac
}

update_gitignore() {
    if [ -f .gitignore ] && ! grep -q "docker-compose.override.yml" .gitignore; then
        echo ".devcontainer/docker-compose.override.yml" >> .gitignore
        log_info "Updated .gitignore"
    fi
}

main() {
    prepare_host_environment
    configure_environment
    update_gitignore
    log_success "Host preparation complete"
}

main "$@"