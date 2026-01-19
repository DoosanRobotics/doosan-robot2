#!/bin/bash

# Create workspace directories
sudo mkdir -p /workspace/doosan_ws/src
# Fix ownership
sudo chown -R $(whoami) /workspace/doosan_ws

cd /workspace/doosan_ws

# Fix GUI permissions and setup
mkdir -p /tmp/runtime-root
chmod 700 /tmp/runtime-root
export XDG_RUNTIME_DIR=/tmp/runtime-root


# Install dependencies from source
sudo apt-get update
rosdep update
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

# Install Doosan emulator if script exists
if [ -f "/workspace/doosan_ws/src/doosan-robot2/install_emulator.sh" ]; then
    echo "Installing Doosan emulator..."
    /workspace/doosan_ws/src/doosan-robot2/install_emulator.sh
fi

# Setup ROS environment
echo "export XDG_RUNTIME_DIR=/tmp/runtime-root" >> ~/.bashrc
echo "export QT_X11_NO_MITSHM=1" >> ~/.bashrc

# Create a script to fix X11 permissions on startup
cat << 'EOF' | sudo tee /usr/local/bin/fix-x11
#!/bin/bash
mkdir -p /tmp/runtime-root
chmod 700 /tmp/runtime-root
export XDG_RUNTIME_DIR=/tmp/runtime-root
EOF

sudo chmod +x /usr/local/bin/fix-x11

echo "Devcontainer setup completed with Docker and GUI support!"
echo "Run 'fix-x11' before launching GUI applications if needed."
echo "You can now use Docker commands and run the Doosan emulator with RViz2."
