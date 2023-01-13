cmake_minimum_required(VERSION 3.8)
project(ariac_human)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(nav2_controller REQUIRED)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

install(DIRECTORY launch models urdf config map rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  launch/stopbase_server.py
  launch/movebase_server.py
  launch/human_listener.py
  launch/robot_listener.py
  launch/snapshot.py
  launch/clockNode.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
