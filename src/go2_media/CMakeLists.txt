cmake_minimum_required(VERSION 3.8)
project(go2_media)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(OpenCV REQUIRED)
find_package(unitree_go REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(camera_info_manager REQUIRED)

# add_executable(publish_front_image src/front_image_publisher.cpp)
add_library(unitree_camera_component SHARED src/front_image_publisher.cpp)

rclcpp_components_register_node(
  unitree_camera_component
  PLUGIN "camera_composable::UnitreeFrontCamera"
  EXECUTABLE UnitreeFrontCamera
)

ament_target_dependencies(
  unitree_camera_component
  unitree_go
  rclcpp
  OpenCV
  sensor_msgs
  cv_bridge
  std_msgs
  rclcpp_components
  camera_info_manager
  rcl_interfaces
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_export_targets(
  export_unitree_camera_component
)

install(
  TARGETS unitree_camera_component
  EXPORT export_unitree_camera_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
