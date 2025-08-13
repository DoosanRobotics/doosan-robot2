FROM osrf/ros:jazzy-desktop

# Arguments
ARG USERNAME=doosan-robotics
ARG USER_UID=1000
ARG USER_GID=1000
ARG WORKSPACE=/workspaces/doosan_ws

# Install basic packages
RUN apt-get update && apt-get install -y \
    sudo \
    curl \
    wget \
    vim \
    git \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Create user
RUN groupadd --gid $USER_GID $USERNAME || true \
    && useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME || true \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install prerequisite packages
RUN apt-get update && apt-get install -y \
    libpoco-dev \
    libyaml-cpp-dev \
    dbus-x11 \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 packages
RUN apt-get update && apt-get install -y \
    ros-jazzy-control-msgs \
    ros-jazzy-realtime-tools \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gazebo-msgs \
    ros-jazzy-moveit-msgs \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-moveit-configs-utils \
    ros-jazzy-moveit-ros-move-group \
    && rm -rf /var/lib/apt/lists/*

# Install Docker for emulator
RUN apt-get update && apt-get install -y \
    ca-certificates \
    gnupg \
    lsb-release \
    && mkdir -p /etc/apt/keyrings \
    && curl -fsSL https://download.docker.com/linux/ubuntu/gpg | gpg --dearmor -o /etc/apt/keyrings/docker.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | tee /etc/apt/sources.list.d/docker.list > /dev/null \
    && apt-get update \
    && apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin \
    && rm -rf /var/lib/apt/lists/*

# Add user to docker group (after user is created)
RUN usermod -aG docker $USERNAME || true

# Setup environment
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/$USERNAME/.bashrc \
    && chown -R $USER_UID:$USER_GID /home/$USERNAME

# Set environment variables
ENV ROS_DOMAIN_ID=29
ENV WORKSPACE=${WORKSPACE}

# Switch to user
USER $USERNAME
WORKDIR /home/$USERNAME

# Default command
CMD ["/bin/bash"]
