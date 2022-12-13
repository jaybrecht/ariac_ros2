cmake_minimum_required(VERSION 3.8)
project(ariac_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/AGVStatus.msg"
  "msg/Model.msg"
  "msg/BreakBeamStatus.msg"
  "msg/ConveyorBeltState.msg"
  "msg/LogicalCameraImage.msg"
  "msg/VacuumGripperState.msg"
  "msg/Part.msg"
  "msg/PartLot.msg"
  "msg/BinInfo.msg"
  "msg/BinParts.msg"
  "msg/ConveyorParts.msg"
  "msg/PartPose.msg"
  "msg/KitTrayPose.msg"
  "msg/KittingPart.msg"
  "msg/KittingTask.msg"
  "msg/AssemblyPart.msg"
  "msg/AssemblyTask.msg"
  "msg/Order.msg"
  "msg/QualityIssue.msg"

)

set(srv_files
  "srv/ChangeGripperColor.srv"
  "srv/ConveyorBeltControl.srv"
  "srv/MoveAGV.srv"
  "srv/VacuumGripperControl.srv"
  "srv/PerformQualityCheck.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}

  DEPENDENCIES
  builtin_interfaces
  std_msgs
  geometry_msgs

  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
