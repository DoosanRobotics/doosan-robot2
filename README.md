

# [Doosan Robotics](http://www.doosanrobotics.com/kr/)<img src="https://user-images.githubusercontent.com/47092672/97660147-142f1f00-1ab4-11eb-9d14-48f30a666cdc.PNG" width="10%" align="right">
[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.png)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)

![images](https://github.com/user-attachments/assets/0e34c651-8434-48d3-b859-082767846b66)

## Overview
    
This package provides the function to control all models of Doosan robots in the ROS2(Humble) environment.


## Installation

### Prerequisites

To utilize the new emulator in virtual mode, **Docker** is required. Install Docker by following the official guide: [Docker Installation for Ubuntu](https://docs.docker.com/engine/install/ubuntu/)

### Required Dependencies

Before installing the package, ensure that the necessary dependencies are installed:

```bash
sudo apt-get update
sudo apt-get install -y libpoco-dev libyaml-cpp-dev wget \
                        ros-humble-control-msgs ros-humble-realtime-tools ros-humble-xacro \
                        ros-humble-joint-state-publisher-gui ros-humble-ros2-control \
                        ros-humble-ros2-controllers ros-humble-gazebo-msgs ros-humble-moveit-msgs \
                        dbus-x11 ros-humble-moveit-configs-utils ros-humble-moveit-ros-move-group \
                        ros-humble-gazebo-ros-pkgs ros-humble-ros-gz-sim ros-humble-ign-ros2-control


```

### Install Gazebo Simulation

```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install -y libignition-gazebo6-dev ros-humble-gazebo-ros-pkgs ros-humble-ros-gz-sim ros-humble-ros-gz
```

### Package Installation

Ensure that you have installed **ros-humble-desktop** using `apt-get`. We recommend placing the package inside:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Clone the required repositories:

```bash
git clone -b humble https://github.com/doosan-robotics/doosan-robot2.git
```

Install dependencies:

```bash
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```

Run the emulator installation script:

```bash
cd ~/ros2_ws/src/doosan-robot2
chmod +x ./install_emulator.sh
sudo ./install_emulator.sh
```

Build the package:

```bash
cd ~/ros2_ws
colcon build
. install/setup.bash
```

To use **ROS2 with Version 3.x Controller**, specify the build option:

```bash
colcon build --cmake-args -DDRCF_VER=3
```

---

## Launch Parameters

### **mode**

- `mode:=real` → Drive a robot in reality (default IP: `192.168.127.100`, port: `12345`)
- `mode:=virtual` → Drive a robot virtually (default IP: `127.0.0.1`, port: `12345`)
  - Emulator starts and terminates automatically with launch lifetime.

### **name**

- Robot namespace (Default: `dsr01`)

### **host**

- IP Address of **Doosan Robotics Controller**
  - Default: `192.168.137.100`
  - Virtual mode: `127.0.0.1`

### **port**

- Port of **Doosan Robotics Controller** (Default: `12345`)

### **model**

- Doosan robot model name

### **color**

- Select `white` or `blue` (Only `white` for E0609)

### **gui**

- Activate/Deactivate GUI (`true`/`false`)

### **gz**

- Activate/Deactivate Gazebo Simulation (`true`/`false`)

---

## Tutorials

### Launch with **Rviz2**

#### **Real Mode**

```bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.137.100 port:=12345 model:=m1013
```

#### **Virtual Mode**

```bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=virtual host:=127.0.0.1 port:=12345 model:=m1013
```

### Launch with **Gazebo Simulation**

#### **Real Mode**

```bash
ros2 launch dsr_bringup2 dsr_bringup2_gazebo.launch.py mode:=real host:=192.168.137.100 model:=m1013
```

#### **Virtual Mode**

```bash
ros2 launch dsr_bringup2 dsr_bringup2_gazebo.launch.py mode:=virtual host:=127.0.0.1 port:=12346 name:=dsr01 x:=0 y:=0
```

To add additional arms for multi-control:

```bash
ros2 launch dsr_bringup2 dsr_bringup2_spawn_on_gazebo.launch.py mode:=virtual host:=127.0.0.1 port:=12347 name:=dsr02 x:=2 y:=2
```

**Note:** Ensure each additional arm has a unique `port`, `name`, and location (`x`, `y`) to avoid collisions in Gazebo.

### Launch with **MoveIt2**

⚠ **Caution:** MoveIt2 requires Controller Version **2.12 or higher**.

#### **Real Mode**

```bash
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py mode:=real model:=m1013 host:=192.168.137.100
```

#### **Virtual Mode**

```bash
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py mode:=virtual model:=m1013 host:=127.0.0.1
```

## Additional Resources

[Demo Video](https://github.com/user-attachments/assets/bd91aea0-b8b6-4ce1-9040-9ab06630edbe)

