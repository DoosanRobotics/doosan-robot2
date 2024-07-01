

# [Doosan Robotics](http://www.doosanrobotics.com/kr/)<img src="https://user-images.githubusercontent.com/47092672/97660147-142f1f00-1ab4-11eb-9d14-48f30a666cdc.PNG" width="10%" align="right">
[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.png)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)
# Overview
    
    This package provides the function to control all models of Doosan robots in the ROS2(Humble) environment.
    
    ※ Currently, ROS2 related packages are being updated rapidly. 
       Doosan packages will also be updated from time to time and features will be upgraded.
 

# Installation 
#### *Doosan Robot ROS2 Package is implemented at ROS2-humble.*
    ### Prerequisite installation elements before package installation
    $ sudo apt-get install libpoco-dev libyaml-cpp-dev
    $ sudo apt-get install ros-humble-control-msgs ros-humble-realtime-tools ros-humble-xacro ros-humble-joint-state-publisher-gui ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-msgs ros-humble-moveit-msgs dbus-x11

    ### install gazebo sim
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    sudo apt-get update
    sudo apt-get install libignition-gazebo6-dev
    sudo apt-get install ros-humble-gazebo-ros-pkgs ros-humble-moveit-msgs ros-humble-ros-gz-sim
    
    ### We assume that you have installed the ros-humble-desktop package using the apt-get command.
    ### We recommand the /home/<user_home>/ros2_ws/src
    $ mkdir -p ~/ros2_ws/src
    $ cd ~/ros2_ws/src
    $ git clone -b humble-devel https://github.com/doosan-robotics/doosan-robot2.git
    $ git clone -b humble https://github.com/ros-controls/gz_ros2_control
    $ rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
    $ cd ~/ros2_ws
    $ colcon build
    $ . install/setup.bash


## Launch Parameters
### *mode*
- Pass __move:=real__ arguement over launch to drive a robot in reality.   
The default IP and port of our robot controller are _192.168.127.100_ and _12345_.

- Pass __move:=virtual__ argument over launch to drive a robot virtually.   
Our emulators are automatically started and terminated according to launch lifetime.   
The default IP and port of the virtual emulator are _127.0.0.1_ and _12345_.   
( Users need to install the emulator by 'install_emulator.sh' )


### *name* 
- Robot Namespace (Default dsr01)

### *host* 
- Ip Address of Doosan Robotics Controller  (Default 192.168.137.100)   
If users would like to launch virtually, you need to specify host:=127.0.0.1 (the emulator is installed in localhost)

### *port* 
- Port of Doosan Robotics Controller (Default 12345)

### *model*
- Model Inforamtion (Doosan robot model name)

### *color*
- Select white/blue (Only white in case of E0609)

### *gui*
- Activate/Deactivate GUI (true/false)

### *gz*
- Activate/Deactivate Gazebo Sim (true/false) 


## Tutorial
[Screencast from 07-01-2024 08:19:33 PM.webm](https://github.com/leeminju531/doosan-robot2/assets/70446214/70ece15c-248e-4e67-bf6f-0b23f07577ff)

---
### Launch With Rviz2
```bash
// Real Mode
// User needs to match host and port according to real controller's
$ ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.137.100 port:=12345 model:=m1013
```

```bash
// Virtual Mode
// User need to specify port (the virtual controller automatically starts on it)
$ ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=virtual host:=127.0.0.1 port:=12345 model:=m1013
```

---
### Launch With Gazebo Sim
```bash
// Real Mode
$ ros2 launch dsr_bringup2 dsr_bringup2_gazebo.launch.py mode:=real host:=192.168.137.100 model:=m1013
```
```bash
// Virtual Mode
$ ros2 launch dsr_bringup2 dsr_bringup2_gazebo.launch.py mode:=virtual host:=127.0.0.1 port:=12346 name:=dsr01 x:=0 y:=0

// Additionally, you can add adittional arms for multi controls by spawning sperate ones. 
// Keep in mind. you need to distinguish 'port' for controller, 'name' for robot namespace, location for loading on gazebo without collisions. 
$ ros2 launch dsr_bringup2 dsr_bringup2_spawn_on_gazebo.launch.py mode:=virtual host:=127.0.0.1 port:=12347 name:=dsr02 x:=2 y:=2
```

---
### Launch With Moveit2
#### Caution : If you use Moveit2 function, Controller version should be required over 2.12.
```bash
$ ros2 launch dsr_bringup2 dsr_bringup2_moveit2.launch.py mode:=real host:=192.168.137.100 model:=m1013
```

## 4. Nvidia Issac Sim (TBD)
