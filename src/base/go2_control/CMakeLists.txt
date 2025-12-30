cmake_minimum_required(VERSION 3.8)
project(go2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include include/common include/nlohmann)
link_directories(src)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)


add_executable(go2_control src/go2_control.cpp src/common/ros2_sport_client.cpp)
add_executable(init_position src/init_position.cpp src/common/ros2_sport_client.cpp)

ament_target_dependencies(go2_control
  rclcpp
  geometry_msgs
  unitree_go
  unitree_api
)

ament_target_dependencies(init_position
  rclcpp
  geometry_msgs
  unitree_go
  unitree_api
)

install(TARGETS
  go2_control DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  init_position DESTINATION lib/${PROJECT_NAME})

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
