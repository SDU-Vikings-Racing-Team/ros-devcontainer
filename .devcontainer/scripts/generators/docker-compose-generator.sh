#!/bin/bash

# DOCKER COMPOSE GENERATOR

# Get current script directory  
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source dependencies from core directory
source "${SCRIPT_DIR}/../core/config-loader.sh"
source "${SCRIPT_DIR}/../core/logger.sh"

# Generate base volume mounts
generate_base_volumes() {
    cat << EOF
      - \${HOME}/.bash_history:${HOME_DIR}/.bash_history:rw
EOF
}

# Generate X11 volume mounts
generate_x11_volumes() {
    cat << EOF
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - \${XAUTHORITY:-\${HOME}/.Xauthority}:${HOME_DIR}/.Xauthority:ro
EOF
}

# Generate complete override file
generate_override_file() {
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

# Create override based on environment features
create_environment_override() {
    local output_file="$1"
    local features="$2"
    
    local include_x11="false"
    if [[ "$features" == *"x11"* ]]; then
        include_x11="true"
        log_info "Including X11 support in override"
    fi
    
    generate_override_file "$output_file" "$include_x11"
}