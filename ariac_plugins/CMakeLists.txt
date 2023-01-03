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
find_package(sensor_msgs REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})

# Vacuum Gripper
add_library(VacuumGripperPlugin SHARED
  src/vacuum_gripper_plugin.cpp
)
target_include_directories(VacuumGripperPlugin PUBLIC include)
ament_target_dependencies(VacuumGripperPlugin
  "gazebo_ros"
  "ariac_msgs"
)
ament_export_libraries(VacuumGripperPlugin)

# Conveyor Belt Plugin
add_library(conveyor_belt_plugin SHARED
  src/conveyor_belt_plugin.cpp
)
target_include_directories(conveyor_belt_plugin PUBLIC include)
ament_target_dependencies(conveyor_belt_plugin
  "gazebo_ros"
  "ariac_msgs"
)
ament_export_libraries(conveyor_belt_plugin)

# AGV Plugin
add_library(agv_plugin SHARED
  src/agv_plugin.cpp
)
target_include_directories(agv_plugin PUBLIC include)
ament_target_dependencies(agv_plugin
  "gazebo_ros"
  "ariac_msgs"
)
ament_export_libraries(agv_plugin)

# Gripper Change Plugin
add_library(GripperColorPlugin SHARED
  src/gripper_color_plugin.cpp
)
target_include_directories(GripperColorPlugin PUBLIC include)
ament_target_dependencies(GripperColorPlugin
  "gazebo_ros"
  "ariac_msgs"
)
ament_export_libraries(GripperColorPlugin)

# Disable Shadows Plugin
add_library(disable_shadows_plugin SHARED
  src/disable_shadows_plugin.cpp
)
target_include_directories(disable_shadows_plugin PUBLIC include)
ament_target_dependencies(disable_shadows_plugin
  "gazebo_ros"
)
ament_export_libraries(disable_shadows_plugin)

# Object Disposal Plugin
add_library(ObjectDisposalPlugin SHARED
  src/object_disposal_plugin.cpp
)
target_include_directories(ObjectDisposalPlugin PUBLIC include)
ament_target_dependencies(ObjectDisposalPlugin
  "gazebo_ros"
)
ament_export_libraries(ObjectDisposalPlugin)

# Vacuum Gripper
add_library(AGVTrayPlugin SHARED
  src/agv_tray_plugin.cpp
)
target_include_directories(AGVTrayPlugin PUBLIC include)
ament_target_dependencies(AGVTrayPlugin
  "gazebo_ros"
  "ariac_msgs"
)
ament_export_libraries(AGVTrayPlugin)

ament_package()

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
    VacuumGripperPlugin
    conveyor_belt_plugin
    agv_plugin
    GripperColorPlugin
    disable_shadows_plugin
    ObjectDisposalPlugin
    AGVTrayPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# install(DIRECTORY models
#   DESTINATION share/${PROJECT_NAME}/
# )