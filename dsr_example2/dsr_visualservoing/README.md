# Overview    
This package provides the visual servoing example using Doosan robots in the ROS 2(Humble) environment.


![Visual_Servoing_FlowChart drawio](https://github.com/user-attachments/assets/51d8284c-c446-4942-a8f7-d9390151b2d0)


---
# Prequiste List
tf-transformations

```bash
sudo apt install ros-humble-tf-transformations
```

---
## Visual Servoing
### Multi Marker
[VisualServoing_demo.webm](https://github.com/user-attachments/assets/f37836d7-59f3-47c1-96fd-a3d5d988b6cc)


### Single Marker & Failed Detecting Marker
https://github.com/user-attachments/assets/3971869b-ab43-466c-80b0-546f305fe547



# How to run visual servoing example
1. Start the Simulation
```bash
ros2 launch dsr_visualservoing dsr_bringup2_visual_servoing_gazebo.launch.py
```

2. Move to the home position
```bash
ros2 run dsr_visualservoing joint90
```

3. Run the visual servoing example, when the cobot reaches the home position ([0, 0, 90, 0, 90, 0]).
```bash
ros2 launch dsr_visualservoing  visual_servoing_gz.launch.py
```


---
# Option

## + How to change simulation environment:
If you want to change simulation environment, then you should chage `visual_servoing.sdf`.

## + Using depth data:
If you want to use depth data, you can utilize the `/rgbd_camera/depth_image` topic.

## + Camera Calibration Method:
1. Modify the `camera_calibration.sdf` file used in `camera_calibration.launch.py`.
It needs to be adjusted to match the performance of the camera being used.

2. Install the necessary packages for camera calibration:
```bash
sudo apt install ros-humble-camera-calibration
sudo apt install ros-humble-image-pipeline
```

3. Run `camera_calibration.launch.py`:
```bash
ros2 launch dsr_visualservoing camera_calibration.launch.py
```

4. Run the Camera Calibration Node:
```bash
ros2 run camera_calibration cameracalibrator --pattern chessboard --size 6x8 --square 0.02 image:=/rgbd_camera/image
```

5. Rotate and move the checkerboard uploaded in Gazebo until the calibration button becomes active.

6. After pressing the calibration button, click the save button and apply the results shown in the terminal (where the Camera Calibration Node was run) to the `rgbd_camera_gz.yaml` file.


---
# Caution
### 1. If the RAM capacity is low, build errors may occur, so please increase the memory capacity by setting up swap.

### 2. If you want to run Gazebo faster, then you should modify `dsr_controller2_gz.yaml`'s update_rate.
(Path: `doosan-robot2/dsr_controller2/config/dsr_controller2_gz.yaml`)

