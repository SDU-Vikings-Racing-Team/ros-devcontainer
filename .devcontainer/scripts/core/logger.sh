#!/bin/bash

# LOGGING UTILITIES

log_with_timestamp() {
    local level="$1"
    shift
    echo "[$(date +'%Y-%m-%d %H:%M:%S')] [$level] $*"
}

log_info() {
    log_with_timestamp "INFO" "$@"
}

log_error() {
    log_with_timestamp "ERROR" "$@" >&2
}

log_success() {
    log_with_timestamp "SUCCESS" "$@"
}

log_warning() {
    log_with_timestamp "WARNING" "$@"
}

log_debug() {
    if [ "${DEBUG:-}" = "true" ]; then
        log_with_timestamp "DEBUG" "$@"
    fi
}