#!/bash

# Source paths
source "$(dirname "${BASH_SOURCE[0]}")/paths.sh"
source "${PATH_CORE}/config-loader.sh"
source "${PATH_CORE}/logger.sh"

has_content() {
    [ -d "$1" ] && [ "$(ls -A "$1" 2>/dev/null)" ]
}

ensure_directory() {
    local dir="$1"
    local description="${2:-$(basename "$dir")}"
    
    if [ ! -d "$dir" ]; then
        if mkdir -p "$dir" 2>/dev/null; then
            log_info "Created directory: $description"
            return 0
        else
            log_warning "Could not create $description (may be mounted or permissions issue)"
            return 1
        fi
    fi
    return 0
}

ensure_workspace_structure() {
    local workspace_root="${1:-$WORKSPACE_ROOT}"
    
    log_info "Ensuring workspace structure..."
    
    IFS=',' read -ra DIRS <<< "$WORKSPACE_SUBDIRS"
    for subdir in "${DIRS[@]}"; do
        ensure_directory "${workspace_root}/${subdir}" "$subdir"
    done
    
    log_success "Workspace structure ready"
}