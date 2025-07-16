######### cmake stuff
cmake_minimum_required(VERSION 3.16)
project(jonny_hardware_interface LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

######### Set Dependencies
set(Dependencies
  ament_cmake
  hardware_interface
  jonny_interfaces
  rclcpp
  pluginlib
  ros2_socketcan
)

######### Find Dependencies
foreach(Dependency IN ITEMS ${Dependencies})
  find_package(${Dependency} REQUIRED)
endforeach()

######### Jonny Robot Controll 
add_executable(jonny_robot_control 
  src/jonny_robot_control.cpp
  src/homing.cpp
  src/can.cpp
  src/joint_control.cpp
  src/motor_control.cpp)
target_include_directories(jonny_robot_control PRIVATE include)
ament_target_dependencies(jonny_robot_control PUBLIC ${Dependencies})

install(
  TARGETS jonny_robot_control
  DESTINATION lib/${PROJECT_NAME})

######### Jonny Hardware Interface
add_library(
  jonny_hardware_interface
  SHARED
  src/jonny_hardware_interface.cpp
  src/jonny_robot_control.cpp
  src/homing.cpp
  src/can.cpp
  src/joint_control.cpp
  src/motor_control.cpp
)
target_include_directories(jonny_hardware_interface PRIVATE include)
ament_target_dependencies(jonny_hardware_interface PUBLIC ${Dependencies})

######### Include
install(
  DIRECTORY include/
  DESTINATION include
)

######### Hardware Interface Stuff
target_compile_definitions(${PROJECT_NAME} PRIVATE "jonny_hardware_interface_BUILDING_DLL")
pluginlib_export_plugin_description_file(hardware_interface jonny_hardware_interface.xml)
install(
  TARGETS jonny_hardware_interface
  EXPORT export_jonny_hardware_interface
)
ament_export_targets(export_jonny_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${Dependencies})
ament_package()
