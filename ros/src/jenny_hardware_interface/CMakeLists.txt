cmake_minimum_required(VERSION 3.16)
project(jenny_hardware_interface LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(Dependencies
  hardware_interface
  rclcpp
  pluginlib
  ros2_socketcan
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${Dependencies})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  jenny_hardware_interface
  SHARED
  src/jenny_can_interface.cpp
)

target_compile_features(jenny_hardware_interface PUBLIC cxx_std_17)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include)

ament_target_dependencies(
  jenny_hardware_interface PUBLIC
  ${Dependencies}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "jenny_hardware_interface_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface jenny_hardware_interface.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS jenny_hardware_interface
  EXPORT export_jenny_hardware_interface
)

## EXPORTS
ament_export_targets(export_jenny_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${Dependencies})
ament_package()
