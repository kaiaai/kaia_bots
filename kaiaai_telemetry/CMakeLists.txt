# https://docs.ros.org/en/galactic/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html
cmake_minimum_required(VERSION 3.8)
project(kaiaai_telemetry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(kaiaai_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(diagnostic_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

add_executable(telem_test_pub src/telem_test_pub.cpp)
ament_target_dependencies(telem_test_pub rclcpp kaiaai_msgs)

add_executable(telem src/telem.cpp)
ament_target_dependencies(telem rclcpp kaiaai_msgs nav_msgs sensor_msgs geometry_msgs tf2_msgs tf2_ros tf2)
# ament_target_dependencies(telem rclcpp kaiaai_msgs nav_msgs sensor_msgs geometry_msgs diagnostic_msgs tf2_msgs tf2_ros tf2)

install(TARGETS
  telem_test_pub
  telem
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
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
