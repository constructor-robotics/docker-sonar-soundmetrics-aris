FROM ros:jazzy-perception
LABEL maintainer="agomezchav@constructor.university"

ARG workspace=ros2ws
ARG rosversion=jazzy

ENV WORKSPACE=${workspace}
ENV ROSVERSION=${rosversion}
RUN echo "Building with ... ROSVERSION=$ROSVERSION"

# Install other necessary packages and dependencies
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -q -y --no-install-recommends \
    apt-utils \
	vim \
	git \
	iputils-ping \
	net-tools \
    ros-${ROSVERSION}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Upgrade everything
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get upgrade -q -y \
   && rm -rf /var/lib/apt/lists/*

# Create initial workspace
RUN mkdir -p /home/${WORKSPACE}/src

# Source ROS and workspace in every bash session
RUN echo "source /opt/ros/${ROSVERSION}/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /home/${WORKSPACE}/install/setup.bash ]; then source /home/${WORKSPACE}/install/setup.bash; fi" >> /root/.bashrc

# Entrypoint runs colcon build on container startup
COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
