#!/usr/bin/env bash
set -e

# Source ROS
source /opt/ros/humble/setup.bash

# Source workspace
if [ -f "/opt/ws/install/setup.bash" ]; then
    source /opt/ws/install/setup.bash
fi

echo "ROS environment loaded"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

exec "$@"