#!/bin/bash

# Source paths
source "$(dirname "${BASH_SOURCE[0]}")/paths.sh"
source "${PATH_CORE}/config-loader.sh"
source "${PATH_CORE}/logger.sh"

# Check if cache file is fresh
is_cache_fresh() {
    local cache_file="$1"
    local max_age="${2:-$CACHE_MAX_AGE}"
    
    if [ -f "$cache_file" ]; then
        local age=$(($(date +%s) - $(stat -c %Y "$cache_file")))
        [ $age -lt $max_age ]
    else
        return 1
    fi
}

# Update APT cache with caching
update_apt_cache() {
    if is_cache_fresh "$APT_CACHE_FILE"; then
        log_info "APT cache recently updated, skipping..."
        return 0
    fi
    
    log_info "Updating APT package lists..."
    if sudo apt-get update; then
        touch "$APT_CACHE_FILE"
        log_success "APT cache updated"
        return 0
    else
        log_error "Failed to update APT cache"
        return 1
    fi
}

# Update rosdep with caching
update_rosdep() {
    if is_cache_fresh "$ROSDEP_CACHE_FILE"; then
        log_info "Rosdep recently updated, skipping..."
        return 0
    fi
    
    log_info "Updating rosdep..."
    if rosdep update; then
        touch "$ROSDEP_CACHE_FILE"
        log_success "Rosdep updated"
        return 0
    else
        log_error "Failed to update rosdep"
        return 1
    fi
}