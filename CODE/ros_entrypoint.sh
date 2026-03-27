#!/bin/bash
set -e

# Detect platform and source appropriate ROS setup
if [ -f "/opt/ros/$ROS_DISTRO/install/setup.bash" ]; then
    # ARM64/Jetson (dustynv image structure)
    source "/opt/ros/$ROS_DISTRO/install/setup.bash"
else
    # AMD64 (standard ROS install)
    source "/opt/ros/$ROS_DISTRO/setup.bash"
fi

# Source ZED ROS2 workspace if it exists
if [ -f "/root/zed_ros2_ws/install/local_setup.bash" ]; then
    source "/root/zed_ros2_ws/install/local_setup.bash"
fi

# Source user workspace if it exists
if [ -f "/workspace/ros_ws/install/setup.bash" ]; then
    source "/workspace/ros_ws/install/setup.bash"
fi

# Set default ROS_DOMAIN_ID if not set
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# Welcome information
echo "=============================================="
echo "Material Transfer Robot - ROS2 Environment"
echo "=============================================="
echo "ROS distro:      $ROS_DISTRO"
echo "DDS middleware:  ${RMW_IMPLEMENTATION:-default}"
echo "ROS Domain ID:   $ROS_DOMAIN_ID"
echo "Workspaces:      $COLCON_PREFIX_PATH"
echo "----------------------------------------------"

# Show ZED info if available
if [ -d "/usr/local/zed" ]; then
    echo "ZED SDK:         $(ls /usr/local/zed/lib/libsl_zed.so 2>/dev/null && echo 'OK' || echo 'NOT FOUND')"
    if command -v ros2 &> /dev/null; then
        echo "ZED ROS2 pkgs:   $(ros2 pkg list 2>/dev/null | grep -c zed || echo '0') packages"
    fi
fi

# Show platform info
echo "----------------------------------------------"
if [ -f "/etc/nv_tegra_release" ]; then
    echo "Platform:        Jetson ($(head -1 /etc/nv_tegra_release | grep -oP 'R\d+.*?REVISION: \d+\.\d+' | tr -d ' '))"
else
    echo "Platform:        Desktop (x86_64)"
fi
echo "Local IPs:       $(hostname -I | tr ' ' '\n' | head -3 | tr '\n' ' ')"
echo "=============================================="

exec "$@"
