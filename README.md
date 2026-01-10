## Run for debug
ros2 topic echo /camera_composable/camera/image_raw

ros2 topic echo /camera_composable/camera/camera_info

ros2 run image_view image_view image:=/image_rect

ros2 run image_view image_view --ros-args -r image:=/camera/image_rect
ros2 run image_view image_view --ros-args -r image:=/camera/image_raw

ros2 run image_proc rectify_node image:=/camera/image_raw camera_info:=/camera/camera_info

## Calibration camera
### Note:
Robot must be stand down to have a smooth view

[Tutorial Link](https://docs.ros.org/en/kilted/p/camera_calibration/doc/tutorial_mono.html); 
[Apriltag generation](https://github.com/AprilRobotics/apriltag-generation)

### Install required packages
sudo apt install ros-humble-camera-ros
sudo apt install ros-humble-image-view
sudo apt install ros-humble-camera-calibration
sudo apt install ros-humble-image-proc

### Control configuration file by absolute path
ros2 run camera_info camera_info_publisher --ros-args -p camera_info_url:="file:///home/q/ros2_ws/read_yaml_parameter/test.yaml"
### Control configuration file by source code
ros2 run camera_info camera_info_publisher --ros-args -p camera_info_url:="package://./test.yaml"

### Calibrate camera
ros2 run camera_calibration cameracalibrator --no-service-check --size 8x6 --square 0.025 image:=/camera/image_raw
ros2 run camera_calibration cameracalibrator --no-service-check --size 6x4 --square 0.037 image:=/camera/image_raw

### Save map
ros2 run nav2_map_server map_saver_cli -f map1 --ros-args -p map_subscribe_transient_local:=true

##### When robot is lying down
- Translation: [0.021, 0.045, 0.632]
- Rotation: in Quaternion (xyzw) [0.934, 0.003, -0.021, 0.356]
- Rotation: in RPY (radian) [2.414, 0.041, -0.010]
- Rotation: in RPY (degree) [138.326, 2.349, -0.572]
- Matrix:
  0.999  0.020 -0.037  0.021
 -0.010 -0.747 -0.665  0.045
 -0.041  0.664 -0.746  0.632
  0.000  0.000  0.000  1.000

#### When robot stand up
- Translation: [0.050, 0.278, 0.580]
- Rotation: in Quaternion (xyzw) [0.990, -0.006, -0.045, -0.137]
- Rotation: in RPY (radian) [-2.867, 0.091, -0.000]
- Rotation: in RPY (degree) [-164.246, 5.188, -0.012]
- Matrix:
  0.996 -0.025 -0.087  0.050
 -0.000 -0.962  0.272  0.278
 -0.090 -0.270 -0.958  0.580
  0.000  0.000  0.000  1.000

- Translation: [0.042, 0.277, 0.578]
- Rotation: in Quaternion (xyzw) [0.991, -0.004, -0.042, -0.129]
- Rotation: in RPY (radian) [-2.883, 0.085, 0.004]
- Rotation: in RPY (degree) [-165.157, 4.853, 0.201]
- Matrix:
  0.996 -0.018 -0.083  0.042
  0.003 -0.967  0.256  0.277
 -0.085 -0.255 -0.963  0.578
  0.000  0.000  0.000  1.000



