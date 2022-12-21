cmake_minimum_required(VERSION 3.8)
project(ariac_sensors)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(camera_info_manager REQUIRED)
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
find_package(orocos_kdl REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(ariac_msgs REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})

add_library(break_beam_sensor SHARED
  src/break_beam_sensor.cpp
)
target_include_directories(break_beam_sensor PUBLIC include)
ament_target_dependencies(break_beam_sensor
  "gazebo_ros"
  "sensor_msgs"
  "ariac_msgs"
)
ament_export_libraries(break_beam_sensor)

add_library(logical_camera SHARED
  src/logical_camera.cpp
)
target_include_directories(logical_camera PUBLIC include)
ament_target_dependencies(logical_camera
  "gazebo_ros"
  "ariac_msgs"
)
ament_export_libraries(logical_camera)

# Quality Control Plugin
add_library(QualityControlPlugin SHARED
  src/quality_control_plugin.cpp
)
target_include_directories(QualityControlPlugin PUBLIC include)
ament_target_dependencies(QualityControlPlugin
  "orocos_kdl"
  "gazebo_ros"
  "ariac_msgs"
)
ament_export_libraries(QualityControlPlugin)

# Ariac Camera Plugin
add_library(AriacCameraPlugin SHARED
  src/ariac_camera_plugin.cpp
)
target_include_directories(AriacCameraPlugin PUBLIC include)
ament_target_dependencies(AriacCameraPlugin
  "gazebo_ros"
  "sensor_msgs"
  "camera_info_manager"
  "image_transport"
)
ament_export_libraries(AriacCameraPlugin)

ament_package()

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
    break_beam_sensor
    logical_camera
    QualityControlPlugin
    AriacCameraPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}/
)