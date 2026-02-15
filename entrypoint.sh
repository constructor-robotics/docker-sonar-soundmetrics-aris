#!/bin/bash
set -e

# Source ROS
source /opt/ros/${ROSVERSION}/setup.bash

# Build the catkin workspace
echo "Building catkin workspace at /home/${WORKSPACE} ..."
cd /home/${WORKSPACE}
catkin_make
echo "Catkin workspace built successfully."

# Source the workspace
source /home/${WORKSPACE}/devel/setup.bash

# Execute the container's command (default: bash)
exec "$@"
