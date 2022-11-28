cmake_minimum_required(VERSION 3.5.0)
project(ariac_gazebo_ros2_control)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(controller_manager REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wconversion -Wno-sign-conversion -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

include_directories(include)
link_directories(
  ${gazebo_dev_LIBRARY_DIRS}
)

# Libraries
add_library(ariac_gazebo_ros2_control SHARED
  src/gazebo_ros2_control_plugin.cpp
)
ament_target_dependencies(ariac_gazebo_ros2_control
  angles
  controller_manager
  gazebo_dev
  gazebo_ros
  hardware_interface
  pluginlib
  rclcpp
  yaml_cpp_vendor
)

## Install
install(TARGETS
  ariac_gazebo_ros2_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(
  include
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_libraries(
  ariac_gazebo_ros2_control
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
