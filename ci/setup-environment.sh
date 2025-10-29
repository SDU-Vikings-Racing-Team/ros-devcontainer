#!/bin/bash

# Environment setup for CI
# Generates docker-compose.override.yml based on environment

set -e

echo "========================================="
echo "Setting up CI environment"
echo "========================================="

export CI=true
export GITHUB_ACTIONS=true

chmod +x .devcontainer/host-setup.sh

# Run host-setup.sh to generate docker-compose.override.yml
echo "Running host-setup.sh..."
bash .devcontainer/host-setup.sh

# Verify the override file was created
if [ ! -f .devcontainer/docker-compose.override.yml ]; then
    echo "ERROR: docker-compose.override.yml was not created"
    exit 1
fi

echo "Override file created successfully:"
cat .devcontainer/docker-compose.override.yml

echo "========================================="
echo "Environment setup complete!"
echo "========================================="