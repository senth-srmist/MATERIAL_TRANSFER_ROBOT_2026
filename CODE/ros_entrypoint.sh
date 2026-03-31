#!/bin/bash
set -e

# Source ROS2 Humble
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Source user workspace if built (failures are non-fatal — workspace may be partially built)
if [ -f "/workspace/ros_ws/install/setup.bash" ]; then
    source /workspace/ros_ws/install/setup.bash 2>/dev/null || \
        echo "[entrypoint] WARNING: workspace source had errors (partial build?), continuing anyway"
fi

# Default to CycloneDDS
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}

# Execute command
exec "$@"
