#!/bin/bash

# Test suite for ROS2 Devcontainer

set -e

echo "========================================="
echo "Testing ROS2 Devcontainer"
echo "========================================="

# Check ROS install
echo ""
echo "[1/5] Checking ROS installation..."
source /opt/ros/${ROS_DISTRO}/setup.bash
echo "ROS_DISTRO: ${ROS_DISTRO}"
ros2 pkg list | head -n 5
echo "ROS2 is available"

# Check workspace structure
echo ""
echo "[2/5] Checking workspace structure..."
ls -la /home/rosdev/ros_ws/

# Check scripts exist
echo ""
echo "[3/5] Checking scripts..."
ls -la /home/rosdev/ros_ws/scripts/
ls -la /home/rosdev/ros_ws/scripts/core/

# Test environment config
echo ""
echo "[4/5] Testing environment..."
source /home/rosdev/ros_ws/scripts/core/config.sh
echo "ROS_DISTRO: $ROS_DISTRO"
echo "WORKSPACE_ROOT: $WORKSPACE_ROOT"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"

# Verify no X11 in CI
echo ""
echo "[5/5] Verifying CI configuration..."
if [ -f /home/rosdev/.Xauthority ]; then
    echo "ERROR: .Xauthority should not exist in CI"
    exit 1
fi

if [ -d /tmp/.X11-unix ]; then
    echo "ERROR: X11 socket should not exist in CI"
    exit 1
fi

echo ""
echo "========================================="
echo "All tests passed!"
echo "========================================="