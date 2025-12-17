# ROS 2 Jazzy + Gazebo Jetty + TurtleBot3 Environment
# For running Task 1 on Windows via Docker

FROM osrf/ros:jazzy-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV TURTLEBOT3_MODEL=waffle_pi
ENV ROS_DOMAIN_ID=30
ENV GZ_SIM_RESOURCE_PATH=/root/.gz/models:$GZ_SIM_RESOURCE_PATH

# Update and install dependencies
RUN apt-get update && apt-get install -y \
    wget \
    curl \
    gnupg \
    lsb-release \
    software-properties-common \
    python3-pip \
    git \
    vim \
    x11-apps \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Add Gazebo repository
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo Jetty and ROS-Gazebo bridge
RUN apt-get update && apt-get install -y \
    gz-harmonic \
    ros-jazzy-ros-gz \
    && rm -rf /var/lib/apt/lists/*

# Install TurtleBot3 packages and teleop
RUN apt-get update && apt-get install -y \
    ros-jazzy-turtlebot3* \
    ros-jazzy-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

# Create Gazebo models directory
RUN mkdir -p /root/.gz/models

# Set up workspace
WORKDIR /root/robotics-tasks

# Source ROS setup in bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "export TURTLEBOT3_MODEL=waffle_pi" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=30" >> /root/.bashrc && \
    echo "export GZ_SIM_RESOURCE_PATH=/root/.gz/models:\$GZ_SIM_RESOURCE_PATH" >> /root/.bashrc

# Copy project files
COPY . /root/robotics-tasks/

# Make scripts executable
RUN chmod +x /root/robotics-tasks/task1/scripts/*.sh

# Default command
CMD ["/bin/bash"]
