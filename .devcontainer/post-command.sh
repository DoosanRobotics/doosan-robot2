#!/bin/bash

# Create workspace directories
mkdir -p /workspaces/doosan_ws/src
cd /workspaces/doosan_ws

# Fix GUI permissions and setup
mkdir -p /tmp/runtime-root
chmod 700 /tmp/runtime-root
export XDG_RUNTIME_DIR=/tmp/runtime-root

# Fix X11 permissions
xhost +local:root 2>/dev/null || true

# Update package lists
apt-get update

# Install Docker CLI for emulator support
apt-get install -y docker.io

# Install GUI and system dependencies
apt-get install -y \
    libpoco-dev \
    libyaml-cpp-dev \
    dbus-x11 \
    wget \
    x11-apps \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri

# Install ROS2 packages
apt-get install -y \
    ros-jazzy-control-msgs \
    ros-jazzy-realtime-tools \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gazebo-msgs \
    ros-jazzy-moveit-msgs \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-moveit-configs-utils \
    ros-jazzy-moveit-ros-move-group

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
echo "export XDG_RUNTIME_DIR=/tmp/runtime-root" >> ~/.bashrc
echo "export QT_X11_NO_MITSHM=1" >> ~/.bashrc

# Create a script to fix X11 permissions on startup
cat > /usr/local/bin/fix-x11 << 'EOF'
#!/bin/bash
xhost +local:root 2>/dev/null || true
mkdir -p /tmp/runtime-root
chmod 700 /tmp/runtime-root
export XDG_RUNTIME_DIR=/tmp/runtime-root
EOF

chmod +x /usr/local/bin/fix-x11

echo "Devcontainer setup completed with Docker and GUI support!"
echo "Run 'fix-x11' before launching GUI applications if needed."
echo "You can now use Docker commands and run the Doosan emulator with RViz2."
