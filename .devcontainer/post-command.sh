#!/bin/bash

# Create workspace directories
mkdir -p /workspaces/doosan_ws/src
cd /workspaces/doosan_ws

# Update package lists
apt-get update

# Install Docker CLI for emulator support
apt-get install -y docker.io

# Install basic dependencies
apt-get install -y \
    libpoco-dev \
    libyaml-cpp-dev \
    dbus-x11 \
    wget \
    ros-jazzy-control-msgs \
    ros-jazzy-realtime-tools \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers

# Update rosdep
rosdep update

# Install dependencies from source
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

# Install Doosan emulator if script exists
if [ -f "/workspaces/doosan_ws/src/doosan-robot2/install_emulator.sh" ]; then
    echo "Installing Doosan emulator..."
    /workspaces/doosan_ws/src/doosan-robot2/install_emulator.sh
fi

# Setup ROS environment
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "if [ -f /workspaces/doosan_ws/install/setup.bash ]; then source /workspaces/doosan_ws/install/setup.bash; fi" >> ~/.bashrc

echo "Devcontainer setup completed with Docker support!"
echo "You can now use Docker commands and run the Doosan emulator."
