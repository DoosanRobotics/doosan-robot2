FROM osrf/ros:lyrical-desktop

# Arguments
ARG USERNAME=doosan-robotics
ARG USER_UID=1000
ARG USER_GID=1000
ARG WORKSPACE=/workspace/doosan_ws

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
RUN if getent passwd $USER_UID > /dev/null; then userdel -r $(getent passwd $USER_UID | cut -d: -f1); fi \
    && if getent group $USER_GID > /dev/null; then groupdel $(getent group $USER_GID | cut -d: -f1); fi \
    && groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME \
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
    ros-lyrical-control-msgs \
    ros-lyrical-realtime-tools \
    ros-lyrical-xacro \
    ros-lyrical-joint-state-publisher-gui \
    ros-lyrical-ros2-control \
    ros-lyrical-ros2-controllers \
    ros-lyrical-moveit-msgs \
    ros-lyrical-ros-gz-sim \
    ros-lyrical-moveit-configs-utils \
    ros-lyrical-moveit-ros-move-group \
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
RUN echo "source /opt/ros/lyrical/setup.bash" >> /home/$USERNAME/.bashrc \
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
