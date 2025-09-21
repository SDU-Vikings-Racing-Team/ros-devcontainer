#!/bin/bash

# This script runs on the host before the container is created
# It prepares the environment and creates override files as needed

set -e

echo "Preparing host environment for devcontainer..."

# Create required directories
mkdir -p bags

# Detect environment and create appropriate override file
if [ -n "$CI" ] || [ -n "$GITHUB_ACTIONS" ]; then
    echo "CI environment detected - using base configuration only"
    # Remove any existing override file in CI
    rm -f .devcontainer/docker-compose.override.yml
else
    echo "Local environment detected - setting up additional features"
    
    # Check for X11 availability
    if [ -n "$DISPLAY" ]; then
        echo "X11 display found: $DISPLAY"
        
        # Allow X server connections from docker (if xhost is available)
        if command -v xhost >/dev/null 2>&1; then
            xhost +local:docker 2>/dev/null || echo "xhost not available or X11 not accessible"
        fi
        
        # Check for XAUTHORITY
        if [ -z "$XAUTHORITY" ]; then
            # Use default location if XAUTHORITY not set
            export XAUTHORITY="$HOME/.Xauthority"
        fi
        
        # Create override file with X11 mounts
        cat > .devcontainer/docker-compose.override.yml << 'EOF'
version: '3.8'

services:
  ros2-dev:
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ${XAUTHORITY:-${HOME}/.Xauthority}:/home/rosdev/.Xauthority:ro
      - ${HOME}/.bash_history:/home/rosdev/.bash_history:rw
EOF
        echo "Created docker-compose.override.yml with X11 support"
    else
        echo "No DISPLAY found - GUI applications will not be available"
        
        # Create minimal override with just bash history
        cat > .devcontainer/docker-compose.override.yml << 'EOF'
version: '3.8'

services:
  ros2-dev:
    volumes:
      - ${HOME}/.bash_history:/home/rosdev/.bash_history:rw
EOF
        echo "Created docker-compose.override.yml without X11"
    fi
    
    # Ensure bash history file exists
    touch "$HOME/.bash_history" 2>/dev/null || true
fi

# Make the override file part of gitignore
if [ -f .gitignore ] && ! grep -q "docker-compose.override.yml" .gitignore; then
    echo ".devcontainer/docker-compose.override.yml" >> .gitignore
fi

echo "Host preparation complete"