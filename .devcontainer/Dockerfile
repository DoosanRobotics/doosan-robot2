
# Build with offical support (https://github.com/osrf/docker_images)
FROM osrf/ros:humble-desktop


# UID specification can be gone in case of starting with devcontainer
# https://github.com/microsoft/vscode-remote-release/issues/4946#issuecomment-1348583560
ARG USERNAME=doosan-robotics
ARG USER_UID=1000
ARG USER_GID=1000
# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  # [Optional] Add sudo support for the non-root user
  && apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  # Cleanup
  && rm -rf /var/lib/apt/lists/* \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc \
  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc

# Default workspace when building with devcontainer.
# see https://code.visualstudio.com/remote/advancedcontainers/change-default-source-mount
ARG WORKSPACE=/workspaces/doosan_ws
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/$USERNAME/.bashrc

# Preferred deps.
RUN apt-get update && apt-get install -y python3-pip \
  && apt-get install -y vim

### Prerequisite installation elements before package installation
RUN apt-get install -y libpoco-dev libyaml-cpp-dev dbus-x11
RUN apt-get install -y ros-humble-control-msgs ros-humble-realtime-tools ros-humble-xacro &&\
  apt-get install -y ros-humble-joint-state-publisher-gui ros-humble-ros2-control  &&\ 
  apt-get install -y ros-humble-ros2-controllers ros-humble-gazebo-msgs ros-humble-moveit-msgs &&\
  apt-get install -y ros-humble-gazebo-ros-pkgs ros-humble-ros-gz-sim ros-humble-moveit* &&\
  apt-get install -y ros-humble-ign-ros2-control

# Dood for emulator
RUN apt-get update && apt-get install -y ca-certificates curl &&\
install -m 0755 -d /etc/apt/keyrings &&\
curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc &&\
chmod a+r /etc/apt/keyrings/docker.asc
RUN echo \
"deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
$(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
tee /etc/apt/sources.list.d/docker.list > /dev/null
RUN apt-get update &&\
apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
RUN usermod -aG docker $USERNAME
RUN newgrp docker


ENV ROS_DOMAIN_ID=29
