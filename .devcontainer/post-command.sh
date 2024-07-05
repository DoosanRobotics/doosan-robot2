mkdir -p /workspace/doosan_ws/src
cd /workspace/doosan_ws
sudo chown doosan-robotics:doosan-robotics . 
sudo chown doosan-robotics:doosan-robotics src

sudo apt-get update
rosdep update
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

/workspace/doosan_ws/src/doosan-robot2/install_emulator.sh
