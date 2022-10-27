cmake_minimum_required(VERSION 3.8)
project(ariac_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ariac_msgs REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})

add_library(vacuum_gripper SHARED
  src/vacuum_gripper.cpp
)
target_include_directories(vacuum_gripper PUBLIC include)
ament_target_dependencies(vacuum_gripper
  "gazebo_ros"
  "ariac_msgs"
)
ament_export_libraries(vacuum_gripper)

ament_package()

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
    vacuum_gripper
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# install(DIRECTORY models
#   DESTINATION share/${PROJECT_NAME}/
# )