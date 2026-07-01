#!/bin/bash
set -e

# Preserve host-mounted library paths before ROS overwrites them
SAVED_LD_PATH="$LD_LIBRARY_PATH"

# Source ROS2 Humble
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Source user workspace
if [ -f "/workspace/ros_ws/install/setup.bash" ]; then
    source /workspace/ros_ws/install/setup.bash 2>/dev/null || \
        echo "[entrypoint] WARNING: workspace source had errors (partial build?), continuing anyway"
fi

# Restore host paths (Tegra, CUDA, cuDNN, ZED) in front of ROS paths
export LD_LIBRARY_PATH="${SAVED_LD_PATH}:${LD_LIBRARY_PATH}"

# Default to CycloneDDS
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}

exec "$@"
