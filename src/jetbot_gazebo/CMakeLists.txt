cmake_minimum_required(VERSION 3.8)
project(jetbot_gazebo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(diff_drive_controller REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)
find_package(ros_gz REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-sim8 REQUIRED)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install world files
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# Install model files
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(PROGRAMS launch/jetbot_gazebo.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
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

ament_export_dependencies(rclcpp geometry_msgs nav_msgs sensor_msgs tf2_ros
                          diff_drive_controller gz_ros2_control ament_cmake_ros
                          rosidl_typesupport_c ros_gz gz-transport13 gz-msgs10 gz-sim8)

ament_package()
