cmake_minimum_required(VERSION 3.8)
project(jonny_servo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# add executable
add_executable(moveit_servo_interface
    src/moveit_servo_interface.cpp)

find_package(moveit_common REQUIRED)
moveit_package()

# find dependencies
set(DEPENDENCIES
  ament_cmake
  geometry_msgs
  Boost
  control_msgs
  Eigen3
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_move_group
  moveit_ros_planning_interface
  moveit_servo
  pluginlib
  rclcpp
  rclcpp_action
  tf2_geometry_msgs
  tf2_geometry_msgs
  tf2_ros
)

foreach(Dependency IN ITEMS ${DEPENDENCIES})
  find_package(${Dependency} REQUIRED COMPONENTS)
endforeach()

ament_export_dependencies(
  ${DEPENDENCIES}
)
ament_target_dependencies(moveit_servo_interface
  ${DEPENDENCIES})

install(TARGETS moveit_servo_interface
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
