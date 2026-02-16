#!/bin/bash
set -e

# Source ROS
source /opt/ros/${ROSVERSION}/setup.bash

# Build the colcon workspace
echo "Building colcon workspace at /home/${WORKSPACE} ..."
cd /home/${WORKSPACE}
colcon build --symlink-install
echo "Colcon workspace built successfully."

# Source the workspace
source /home/${WORKSPACE}/install/setup.bash

# Execute the container's command (default: bash)
exec "$@"
