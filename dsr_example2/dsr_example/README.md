<!-- DSR Test Package Description -->
# DSR ROS 2 dsr_example Package

## Description

A simple dsr_example package for using a Doosan Robot with the DSR ROS 2 `dsr_common2` Python package.

- **dance example**: Example using various motion services (`movej`, `movel`, `moveb`, etc.)
- **single_robot_simple example**: Example using `movej` motion services

## Requirements

- ROS 2 Humble release
- DSR ROS 2 Package Setting

## Build instructions

### DSR ROS 2 dsr_common2 package install

Add the following code to `dsr_common2/CMakeLists.txt`:

```cmake
install(DIRECTORY imp DESTINATION lib/${PROJECT_NAME}
FILES_MATCHING PATTERN "*.py"
PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_READ WORLD_READ
)
```

### Edit .bashrc
Add Python Path for DSR module:
```bash
export PYTHONPATH=$PYTHONPATH:~/ros2_ws/install/dsr_common2/lib/dsr_common2/imp
```

### Build
Build dsr_example package:
```shell
cd ros2_ws
colcon build --packages-select dsr_example
```

### Run Dance Example
```shell
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=virtual host:=127.0.0.1 port:=12345 model:=m1013
ros2 run dsr_example dance
```
[Dance Example 시연 영상.webm](https://github.com/user-attachments/assets/19cf76fe-81c6-442c-90ae-ef4b49af17c0)


### Run Single Robot Simple Example
```shell
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=virtual host:=127.0.0.1 port:=12345 model:=m1013
ros2 run dsr_example single_robot_simple
```

[Screencast from 2024년 09월 26일 13시 21분 53초.webm](https://github.com/user-attachments/assets/c9b19447-0f7e-42b2-824a-a744db24079e)

### Run Hold-to-Run (H2R) Action Examples
```shell
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=virtual host:=127.0.0.1 port:=12345 model:=m1013
ros2 run dsr_example test_action_movej_h2r
ros2 run dsr_example test_action_movel_h2r
ros2 run dsr_example test_action_jog_h2r
```

The `motion/movej_h2r`, `motion/movel_h2r` and `motion/jog_h2r` actions move the robot only while the client holds the motion. An H2R action client must publish its goal UUID (`unique_identifier_msgs/msg/UUID`, the same id it passes as `goal_uuid` when sending the goal) on `motion/h2r_keepalive` at 10 Hz or faster for as long as it wants the robot to move. If no keep-alive for the goal arrives within `h2r_keepalive_timeout_ms` (default 200 ms, controller parameter), the robot is stopped and the goal is aborted. Only one H2R goal can be active at a time. The examples above show the pattern.
