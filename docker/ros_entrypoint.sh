#!/bin/bash
set -e

# Source ROS 2 Humble
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source workspace if it exists
if [ -f "${WORKSPACE}/install/setup.bash" ]; then
    source ${WORKSPACE}/install/setup.bash
fi

# Set default ROS_DOMAIN_ID if not set
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# Use CycloneDDS for better performance
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}

# Execute the command
exec "$@"
