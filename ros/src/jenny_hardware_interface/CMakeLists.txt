######### cmake stuff
cmake_minimum_required(VERSION 3.16)
project(jenny_hardware_interface LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

######### Set Dependencies
set(Dependencies
  ament_cmake
  hardware_interface
  jenny_interfaces
  rclcpp
  pluginlib
  ros2_socketcan
)

######### Find Dependencies
foreach(Dependency IN ITEMS ${Dependencies})
  find_package(${Dependency} REQUIRED)
endforeach()

######### Jenny Motor Controll 
add_executable(jenny_motor_control 
  src/jenny_motor_control.cpp
  src/homing.cpp
  src/can.cpp
  src/motor_control.cpp)
target_include_directories(jenny_motor_control PRIVATE include)
ament_target_dependencies(jenny_motor_control PUBLIC ${Dependencies})

install(
  TARGETS jenny_motor_control
  DESTINATION lib/${PROJECT_NAME})

######### Jenny Hardware Interface
add_library(
  jenny_hardware_interface
  SHARED
  src/jenny_hardware_interface.cpp
)
target_include_directories(jenny_hardware_interface PRIVATE include)
ament_target_dependencies(jenny_hardware_interface PUBLIC ${Dependencies})

######### Include
install(
  DIRECTORY include/
  DESTINATION include
)

######### Hardware Interface Stuff
target_compile_definitions(${PROJECT_NAME} PRIVATE "jenny_hardware_interface_BUILDING_DLL")
pluginlib_export_plugin_description_file(hardware_interface jenny_hardware_interface.xml)
install(
  TARGETS jenny_hardware_interface
  EXPORT export_jenny_hardware_interface
)
ament_export_targets(export_jenny_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${Dependencies})
ament_package()
