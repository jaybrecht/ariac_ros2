#include <memory>
#include <chrono>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <ariac_msgs/srv/change_gripper.hpp>

#include "rclcpp/rclcpp.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "floor_robot_moveit_test");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("gripper_change");

  // Create tf listener
  std::unique_ptr<tf2_ros::Buffer> tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  moveit::planning_interface::MoveGroupInterface::Options floor_opt(
    "floor_robot_whole", "floor_robot_description", "/floor_robot");

  moveit::planning_interface::MoveGroupInterface floor_move_group_interface(node, floor_opt);

  floor_move_group_interface.setMaxVelocityScalingFactor(1.0);
  floor_move_group_interface.setMaxAccelerationScalingFactor(1.0);

  floor_move_group_interface.setPlanningTime(1.0);

  // Move to home position
  floor_move_group_interface.setNamedTarget("home");

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(floor_move_group_interface.plan(plan));

  if (success) {
    floor_move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Unable to generate plan");
  }

  auto joint_names = floor_move_group_interface.getActiveJoints();

  for (auto name: joint_names) {
    RCLCPP_INFO_STREAM(logger, name);
  }

  // Move to end of linear rail 
  floor_move_group_interface.setJointValueTarget("linear_actuator", 4.5);
  floor_move_group_interface.setJointValueTarget("floor_shoulder_pan_joint", 1.5);

  success = static_cast<bool>(floor_move_group_interface.plan(plan));

  if (success) {
    floor_move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Unable to generate plan");
  }

  // Lookup tf frame for gripper station 
  geometry_msgs::msg::TransformStamped t;

  try {
    t = tf_buffer->lookupTransform("world", "kts1_tool_changer_trays_frame", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(logger, "Could not get transform ");
    return 0;
  }

  geometry_msgs::msg::Pose above_tray_tool_changer;
  geometry_msgs::msg::Pose at_tray_tool_changer;

  std::vector<geometry_msgs::msg::Pose> waypoints;
  
  above_tray_tool_changer = floor_move_group_interface.getCurrentPose().pose;

  above_tray_tool_changer.position.x = t.transform.translation.x;
  above_tray_tool_changer.position.y = t.transform.translation.y;

  waypoints.push_back(above_tray_tool_changer);

  at_tray_tool_changer = floor_move_group_interface.getCurrentPose().pose;
  at_tray_tool_changer.position.x = t.transform.translation.x;
  at_tray_tool_changer.position.y = t.transform.translation.y;
  at_tray_tool_changer.position.z = t.transform.translation.z;

  waypoints.push_back(at_tray_tool_changer);

  moveit_msgs::msg::RobotTrajectory trajectory;
  floor_move_group_interface.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

  // Retime trajectory 
  robot_trajectory::RobotTrajectory rt(floor_move_group_interface.getCurrentState()->getRobotModel(), "floor_robot_whole");
  rt.setRobotTrajectoryMsg(*floor_move_group_interface.getCurrentState(), trajectory);
  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  totg.computeTimeStamps(rt, 0.1, 0.1);
  rt.getRobotTrajectoryMsg(trajectory);

  floor_move_group_interface.execute(trajectory);

  // Call service to change gripper type
  rclcpp::Client<ariac_msgs::srv::ChangeGripper>::SharedPtr change_gripper =
    node->create_client<ariac_msgs::srv::ChangeGripper>("/ariac/floor_robot_change_gripper");

  auto request = std::make_shared<ariac_msgs::srv::ChangeGripper::Request>();
  request->gripper_type = ariac_msgs::srv::ChangeGripper::Request::TRAY_GRIPPER;

  // while (!change_gripper->wait_for_service()) {
  //   if (!rclcpp::ok()) {
  //     RCLCPP_ERROR(logger, "Interrupted while waiting for the service. Exiting.");
  //     return 0;
  //   }
  //   RCLCPP_INFO(logger, "service not available, waiting again...");
  // }

  auto result = change_gripper->async_send_request(request);

  executor.remove_node(node);  

  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    if (!result.get()->success) {
      RCLCPP_INFO_STREAM(logger, result.get()->message);
    } else {
      RCLCPP_INFO(logger, "Changed gripper to tray_gripper");
    }
  } else {
    RCLCPP_ERROR(logger, "Failed to call service");
  }

  executor.add_node(node);

  waypoints.clear();
  waypoints.push_back(above_tray_tool_changer);

  floor_move_group_interface.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

  rt.setRobotTrajectoryMsg(*floor_move_group_interface.getCurrentState(), trajectory);
  totg.computeTimeStamps(rt, 0.1, 0.1);
  rt.getRobotTrajectoryMsg(trajectory);

  floor_move_group_interface.execute(trajectory);

  // Switch back to parts gripper
  try {
    t = tf_buffer->lookupTransform("world", "kts1_tool_changer_parts_frame", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(logger, "Could not get transform ");
    return 0;
  }

  geometry_msgs::msg::Pose above_parts_tool_changer;
  geometry_msgs::msg::Pose at_parts_tool_changer;
  
  above_parts_tool_changer = floor_move_group_interface.getCurrentPose().pose;

  above_parts_tool_changer.position.x = t.transform.translation.x;
  above_parts_tool_changer.position.y = t.transform.translation.y;

  waypoints.clear();
  waypoints.push_back(above_parts_tool_changer);

  at_parts_tool_changer = floor_move_group_interface.getCurrentPose().pose;
  at_parts_tool_changer.position.x = t.transform.translation.x;
  at_parts_tool_changer.position.y = t.transform.translation.y;
  at_parts_tool_changer.position.z = t.transform.translation.z;

  waypoints.push_back(at_parts_tool_changer);

  floor_move_group_interface.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

  rt.setRobotTrajectoryMsg(*floor_move_group_interface.getCurrentState(), trajectory);
  totg.computeTimeStamps(rt, 0.1, 0.1);
  rt.getRobotTrajectoryMsg(trajectory);

  floor_move_group_interface.execute(trajectory);

  request->gripper_type = ariac_msgs::srv::ChangeGripper::Request::PART_GRIPPER;

  result = change_gripper->async_send_request(request);

  executor.remove_node(node);  

  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    if (!result.get()->success) {
      RCLCPP_INFO_STREAM(logger, result.get()->message);
    } else {
      RCLCPP_INFO(logger, "Changed gripper to parts_gripper");
    }
  } else {
    RCLCPP_ERROR(logger, "Failed to call service");
  }

  executor.add_node(node);

  waypoints.clear();
  waypoints.push_back(above_parts_tool_changer);

  floor_move_group_interface.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

  rt.setRobotTrajectoryMsg(*floor_move_group_interface.getCurrentState(), trajectory);
  totg.computeTimeStamps(rt, 0.1, 0.1);
  rt.getRobotTrajectoryMsg(trajectory);

  floor_move_group_interface.execute(trajectory);

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}