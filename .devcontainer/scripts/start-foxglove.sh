#!/bin/bash

# FOXGLOVE BRIDGE LAUNCHER

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/core/config.sh"
source "${SCRIPT_DIR}/core/logger.sh"

PORT="${FOXGLOVE_PORT:-8765}"
ADDRESS="${FOXGLOVE_ADDRESS:-0.0.0.0}"

while [[ $# -gt 0 ]]; do
    case $1 in
        --port|-p)
            PORT="$2"
            shift 2
            ;;
        --address|-a)
            ADDRESS="$2"
            shift 2
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  -p, --port PORT        WebSocket port (default: 8765)"
            echo "  -a, --address ADDRESS  Bind address (default: 0.0.0.0)"
            echo "  -h, --help            Show this help message"
            exit 0
            ;;
        *)
            log_error "Unknown option: $1"
            exit 1
            ;;
    esac
done

start_bridge() {
    # Validate package exists
    if [ -f "${ROS_ROOT}/setup.bash" ]; then
        source "${ROS_ROOT}/setup.bash"
    else
        log_error "ROS setup not found"
        return 1
    fi
    
    if ! ros2 pkg list | grep -q foxglove_bridge; then
        log_error "foxglove_bridge package not found"
        log_info "Install with: sudo apt install ros-${ROS_DISTRO}-foxglove-bridge"
        return 1
    fi
    
    # Source workspace
    if [ -f "${WORKSPACE_ROOT}/install/setup.bash" ]; then
        source "${WORKSPACE_ROOT}/install/setup.bash"
    fi
    
    log_info "Starting Foxglove Bridge on port ${PORT}"
    
    ros2 run foxglove_bridge foxglove_bridge \
        --ros-args \
        -p port:=${PORT} \
        -p address:=${ADDRESS}
}

start_bridge