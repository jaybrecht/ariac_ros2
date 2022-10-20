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

add_library(ariac_plugin_template SHARED
  src/ariac_plugin_template.cpp
)
target_include_directories(ariac_plugin_template PUBLIC include)
ament_target_dependencies(ariac_plugin_template
  "gazebo_ros"
  "ariac_msgs"
)
ament_export_libraries(ariac_plugin_template)

ament_package()

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
    ariac_plugin_template
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# install(DIRECTORY models
#   DESTINATION share/${PROJECT_NAME}/
# )