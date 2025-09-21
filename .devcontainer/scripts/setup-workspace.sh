#!/bin/bash

# Setup workspace structure and symlinks

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/utils.sh"

log_info "Setting up workspace structure..."

# Ensure all workspace directories exist
for dir in "${WS_DIRS[@]}"; do
    ensure_dir "$dir"
done

# Link host source packages if they exist
if [ -d "${HOST_MOUNT}/src" ] && has_content "${HOST_MOUNT}/src"; then
    log_info "Linking host packages..."
    
    # Create symlinks for each package in host src
    for package in "${HOST_MOUNT}/src"/*; do
        if [ -d "$package" ]; then
            package_name=$(basename "$package")
            target="${WS_HOST_PACKAGES}/${package_name}"
            
            if [ ! -e "$target" ]; then
                ln -s "$package" "$target"
                log_info "Linked: $package_name"
            else
                log_info "Already linked: $package_name"
            fi
        fi
    done
    
    log_success "Host packages linked"
else
    log_info "No host packages to link"
fi

# Import third-party packages if repos file exists and has content
REPOS_FILE="${WS_ROOT}/packages.repos"
if [ -f "$REPOS_FILE" ]; then
    # Check if file has repository entries (not just comments)
    if grep -qvE '^\s*(#|$)' "$REPOS_FILE" && grep -q 'type:' "$REPOS_FILE"; then
        log_info "Importing third-party packages..."
        cd "${WS_THIRDPARTY}"
        
        if command -v vcs &> /dev/null; then
            vcs import < "$REPOS_FILE"
            log_success "Third-party packages imported"
        else
            log_error "vcs command not found. Installing python3-vcstool..."
            sudo apt-get update && sudo apt-get install -y python3-vcstool
            vcs import < "$REPOS_FILE"
            log_success "Third-party packages imported"
        fi
    else
        log_info "packages.repos contains no active repositories"
    fi
else
    log_info "No packages.repos file found"
fi

log_success "Workspace structure ready"