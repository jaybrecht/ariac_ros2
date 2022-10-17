cmake_minimum_required(VERSION 3.8)
project(ariac_sensors)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(ariac_msgs REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})

add_library(gazebo_ros_break_beam_sensor SHARED
  src/gazebo_ros_break_beam_sensor.cpp
)
target_include_directories(gazebo_ros_break_beam_sensor PUBLIC include)
ament_target_dependencies(gazebo_ros_break_beam_sensor
  "gazebo_ros"
  "sensor_msgs"
  "ariac_msgs"
)
ament_export_libraries(gazebo_ros_break_beam_sensor)

add_library(gazebo_ros_logical_camera SHARED
  src/gazebo_ros_logical_camera.cpp
)
target_include_directories(gazebo_ros_logical_camera PUBLIC include)
ament_target_dependencies(gazebo_ros_logical_camera
  "gazebo_ros"
  "ariac_msgs"
)
ament_export_libraries(gazebo_ros_logical_camera)

ament_package()

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
    gazebo_ros_break_beam_sensor
    gazebo_ros_logical_camera
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}/
)