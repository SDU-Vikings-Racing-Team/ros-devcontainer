#!/bin/bash

# Setup shell environment and aliases

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/utils.sh"

log_info "Configuring shell environment..."

# Create bashrc additions if not already present
if ! grep -q "# ROS2 Environment Setup" ~/.bashrc; then
    cat >> ~/.bashrc << 'EOF'

# ROS2 Environment Setup
if [ -f /home/rosdev/ros_ws/scripts/config.sh ]; then
    source /home/rosdev/ros_ws/scripts/config.sh
fi

if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then
    source /opt/ros/${ROS_DISTRO}/setup.bash
fi

if [ -f /home/rosdev/ros_ws/install/setup.bash ]; then
    source /home/rosdev/ros_ws/install/setup.bash
fi

# Development Aliases
alias cb='cd ${WS_ROOT} && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'
alias cbp='cd ${WS_ROOT} && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select'
alias cbu='cd ${WS_ROOT} && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to'
alias ct='cd ${WS_ROOT} && colcon test'
alias ctp='cd ${WS_ROOT} && colcon test --packages-select'
alias ctr='cd ${WS_ROOT} && colcon test-result --verbose'
alias source_ws='source ${WS_ROOT}/install/setup.bash'
alias clean_ws='cd ${WS_ROOT} && rm -rf build install log'
alias rosdep_install='cd ${WS_ROOT} && rosdep install --from-paths src --ignore-src -r -y'

# Shell Aliases
alias ll='ls -alF --color=auto'
alias la='ls -A --color=auto'
alias l='ls -CF --color=auto'

# Auto-cd to workspace
cd ${WS_ROOT}
EOF
    log_success "Shell environment configured"
else
    log_info "Shell environment already configured"
fi