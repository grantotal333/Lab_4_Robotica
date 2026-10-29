
cmake_minimum_required(VERSION 3.8)
project(ur5_moveit_control)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(
  ur5_moveit_node
  src/ur5_moveit_node.cpp
)

ament_target_dependencies(
  ur5_moveit_node
  rclcpp
  std_msgs
  moveit_ros_planning_interface
)

install(
  TARGETS ur5_moveit_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
