# Use Doosan robots with MuJoCo

![Doosan robot in MuJoCo](dsr_mujoco/doc/images/m1013_in_mujoco.png)

## Overview

This package enables controlling Doosan robots in MuJoCo simulation using ROS 2.  
Supported models:
- **m1013** (white series)  
- **m0609** (white series)

## Prerequisites

1. Install MuJoCo from Google DeepMind:  
   - Official [Releases](https://github.com/google-deepmind/mujoco/releases)  
   - Website: https://mujoco.org/  

2. Set `MUJOCO_DIR` (update path & version) in your `~/.bashrc`:
   ```bash
   export MUJOCO_DIR=/PATH/TO/MUJOCO/mujoco-3.x.x
   ```

3. Install MuJoCo ROS 2 control package:
   ```bash
   git clone https://github.com/moveit/mujoco_ros2_control.git
   cd mujoco_ros2_control
   git checkout bd2d576331a1be2d2701f35f1b736297d09f1cea
   colcon build
   source install/setup.bash
   ```
   *Note:* Recent versions have simtime issue. Extra change or tuning may be required.  
   
   *Tip:* add to your `~/.bashrc`:
   ```bash
   source ~/mujoco_ros2_control/install/local_setup.bash
   ```

## Launch & Demo

1. **Launch Blank scene (m1013):**  
   ```bash
   source /PATH/TO/DOOSAN_WS/install/setup.bash
   ros2 launch dsr_bringup2 dsr_bringup2_mujoco.launch.py
   ```
2. In another terminal, run demo movements for test:
   ```bash
   ros2 run dsr_example dance
   ```

1. **Slope demo (m1013 + 2F85 gripper)**  
   ```bash
   ros2 launch dsr_bringup2 dsr_bringup2_mujoco_gripper.launch.py \
     scene_path:=demo/slope_demo_scene.xml
   ```
2. In another terminal, run Pick & Place movements:  
   ```bash
   ros2 run dsr_example slope_demo
   ```

### Demo Video
[![Watch on YouTube](dsr_mujoco/doc/images/m1013_with_slide.png)](https://youtu.be/Jqaam6x79t4)

## Future Work

1. Model compatible with MJX will be upload on Google DeepMind [menagerie](https://github.com/google-deepmind/mujoco_menagerie) soon. (Not officially maintained)
2. RL demo tasks on DeepMind [playground](https://github.com/google-deepmind/mujoco_playground) will be provided laterly. (Not officially maintained)
