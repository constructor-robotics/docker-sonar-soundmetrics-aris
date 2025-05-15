FROM ros:noetic-perception
LABEL maintainer="agomezchav@constructor.university"

ARG workspace=catkin_ws
ARG rosversion=melodic

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
    && rm -rf /var/lib/apt/lists/*

# Upgrade everything
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get upgrade -q -y \
   && rm -rf /var/lib/apt/lists/*

# Create initial workspace 
RUN mkdir -p /home/${WORKSPACE}/src
RUN /bin/bash -c ". /opt/ros/${ROSVERSION}/setup.bash; catkin_init_workspace /home/${WORKSPACE}/src"  
RUN /bin/bash -c ". /opt/ros/${ROSVERSION}/setup.bash; cd /home/${WORKSPACE}; catkin_make"

CMD ["/bin/bash", "-c", "source /opt/ros/${ROSVERSION}/setup.bash; source /home/${WORKSPACE}/devel/setup.bash; exec bash"]