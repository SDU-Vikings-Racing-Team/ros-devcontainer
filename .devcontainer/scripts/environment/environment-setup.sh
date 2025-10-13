#!/bin/bash

# SHELL ENVIRONMENT SETUP

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/../core/config-loader.sh"
source "${SCRIPT_DIR}/../core/logger.sh"

# Generate shell aliases - this is a bit hacky and not very pretty
generate_shell_aliases() {
    cat << EOF

# ROS2 Environment Setup
if [ -f ${CONFIG_DIR}/defaults.env ]; then
    set -a
    source ${CONFIG_DIR}/defaults.env
    set +a
fi

if [ -f \${ROS_ROOT}/setup.bash ]; then
    source \${ROS_ROOT}/setup.bash
fi

if [ -f ${WORKSPACE_ROOT}/install/setup.bash ]; then
    source ${WORKSPACE_ROOT}/install/setup.bash
fi

# ROS Aliases
alias cb='cd \${WORKSPACE_ROOT} && colcon build \${COLCON_BUILD_ARGS}'
alias cbp='cd \${WORKSPACE_ROOT} && colcon build \${COLCON_BUILD_ARGS} --packages-select'
alias cbu='cd \${WORKSPACE_ROOT} && colcon build \${COLCON_BUILD_ARGS} --packages-up-to'
alias ct='cd \${WORKSPACE_ROOT} && colcon test'
alias ctp='cd \${WORKSPACE_ROOT} && colcon test --packages-select'
alias ctr='cd \${WORKSPACE_ROOT} && colcon test-result --verbose'
alias source_ws='source \${WORKSPACE_ROOT}/install/setup.bash'
alias clean_ws='cd \${WORKSPACE_ROOT} && rm -rf build install log'
alias rosdep_install='cd \${WORKSPACE_ROOT} && rosdep install --from-paths src --ignore-src -r -y'

# Shell Aliases
alias ll='ls -alF --color=auto'
alias la='ls -A --color=auto'
alias l='ls -CF --color=auto'

# Auto-cd to workspace
cd \${WORKSPACE_ROOT}
EOF
}

# Setup shell environment
setup_shell_environment() {
    log_info "Configuring shell environment..."
    
    if ! grep -q "# ROS2 Environment Setup" ~/.bashrc; then
        generate_shell_aliases >> ~/.bashrc
        log_success "Shell environment configured"
    else
        log_info "Shell environment already configured"
    fi
}

# Execute if called directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    setup_shell_environment
fi