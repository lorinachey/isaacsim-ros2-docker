# Use Ubuntu 22.04 as the base image
FROM ubuntu:22.04

# Set environment variables for non-interactive installation and ROS2
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install necessary packages
RUN apt-get update && apt-get install -y \
    locales \
    apt-utils \
    curl \
    iproute2 \
    gnupg2 \
    lsb-release \
    software-properties-common \
    x11-apps \
    mesa-utils \
    libgl1-mesa-glx \
    libxkbcommon-x11-0 \
    libx11-dev

# Set locale
RUN locale-gen en_US en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# Add ROS2 GPG key and repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add -
RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Update and install ROS2 packages
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-gazebo-* \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3* \
    ros-humble-dynamixel-sdk \
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3 \
    ros-humble-octomap \
    ros-humble-octomap-ros \
    ros-humble-octomap-msgs \
    ros-humble-nav-msgs \
    ros-humble-pcl-ros \
    ros-humble-tf2-tools \
    ros-humble-rqt \
    ros-humble-rqt-graph \
    ros-humble-rqt-tf-tree \
    ros-humble-rqt-plot \
    ros-humble-rqt-console \
    ros-humble-rqt-reconfigure

# Update and install python tools
RUN apt-get update && apt-get install -y \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep \
    vim \
    git \
    python3-pip \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Install joystick-related packages
RUN apt-get update && apt-get install -y \
    joystick \
    jstest-gtk \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    && rm -rf /var/lib/apt/lists/*

# Set up udev rules for joystick devices
RUN echo 'KERNEL=="js[0-9]*", MODE="0666"' > /etc/udev/rules.d/99-joystick.rules

# Set up ROS2 environment and initialize rosdep
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && rosdep init && rosdep update"

# Set TURTLEBOT3_MODEL environment variable
RUN echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
RUN echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models" >> ~/.bashrc
RUN /bin/bash -c "source ~/.bashrc"

# Clone TurtleBot3 simulation repository
WORKDIR /root
RUN git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

# Build TurtleBot3 simulation packages
WORKDIR /root/turtlebot3_simulations
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build"

# Find and set the ROS2 bridge variables for Isaac Sim
RUN echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
RUN echo "export export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/isaac-sim/exts/omni.isaac.ros2_bridge/humble/lib" >> ~/.bashrc
RUN echo "export AMENT_PREFIX_PATH=/opt/ros/humble" >> ~/.bashrc
RUN echo "source ~/.bashrc"

# Copy the fastdds.xml file to the appropriate directory and set permissions on it
RUN mkdir -p ~/.ros
COPY fastdds.xml /root/.ros/fastdds.xml
RUN chmod 644 /root/.ros/fastdds.xml

# Set the environment variable for FASTRTPS_DEFAULT_PROFILES_FILE
RUN echo "export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds.xml" >> ~/.bashrc

# Clone octomap_mapping and rviz plugins repos on ros2 branch
WORKDIR /root/ros2_ws/src
RUN git clone https://github.com/lorinachey/octomap_mapping.git -b ros2
RUN git clone https://github.com/OctoMap/octomap_rviz_plugins.git -b ros2

# Build octomap related packages
WORKDIR /root/ros2_ws
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build"

# Copy over the teleop twist parameters for the joystick controller
COPY teleop_twist_joy.config.yaml /opt/ros/$ROS_DISTRO/share/teleop_twist_joy/config/
RUN chmod 644 /opt/ros/$ROS_DISTRO/share/teleop_twist_joy/config/teleop_twist_joy.config.yaml

# Source the workspace
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# Create a script to launch the simulation
RUN echo '#!/bin/bash' > /root/start_simulation.sh \
    && echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> /root/start_simulation.sh \
    && echo 'export TURTLEBOT3_MODEL=waffle' >> /root/start_simulation.sh \
    && echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models' >> /root/start_simulation.sh \
    && echo 'ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False' >> /root/start_simulation.sh


RUN chmod +x /root/start_simulation.sh

# Set the entrypoint to the ROS2 setup script
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && bash"]

# Set the default command to run when the container starts
CMD ["bash"]

# Expose the display variable for GUI applications
ENV DISPLAY=:0