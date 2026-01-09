
ros2 topic echo /camera_composable/camera/image_raw

ros2 topic echo /camera_composable/camera/camera_info

ros2 run image_view image_view image:=/image_rect

ros2 run image_view image_view --ros-args -r image:=/camera/image_rect
ros2 run image_view image_view --ros-args -r image:=/camera/image_raw

ros2 run image_proc rectify_node image:=/camera/image_raw camera_info:=/camera/camera_info