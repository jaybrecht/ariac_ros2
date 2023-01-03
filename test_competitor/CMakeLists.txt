cmake_minimum_required(VERSION 3.5)
project(test_competitor)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ariac_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(moveit_test src/moveit_test.cpp)
ament_target_dependencies(moveit_test 
  rclcpp 
  moveit_ros_planning_interface
  )

add_executable(gripper_change src/gripper_change.cpp)
ament_target_dependencies(gripper_change 
  rclcpp 
  moveit_ros_planning_interface
  ariac_msgs)

# add_executable(pick_test src/pick_test.cpp)
# ament_target_dependencies(pick_test rclcpp moveit_ros_planning_interface ariac_msgs)

install(TARGETS
  moveit_test
  gripper_change
  # pick_test
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch meshes rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()