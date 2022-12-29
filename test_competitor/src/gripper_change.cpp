#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <sensor_msgs/msg/joint_state.hpp>

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
    t = tf_buffer->lookupTransform("world", "kts1_tool_changer_parts_frame", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(logger, "Could not get transform ");
    return 0;
  }

  geometry_msgs::msg::Pose above_tool_changer;
  geometry_msgs::msg::Pose at_tool_changer;

  std::vector<geometry_msgs::msg::Pose> waypoints;
  
  above_tool_changer = floor_move_group_interface.getCurrentPose().pose;

  above_tool_changer.position.x = t.transform.translation.x;
  above_tool_changer.position.y = t.transform.translation.y;

  waypoints.push_back(above_tool_changer);

  at_tool_changer = floor_move_group_interface.getCurrentPose().pose;
  at_tool_changer.position.x = t.transform.translation.x;
  at_tool_changer.position.y = t.transform.translation.y;
  at_tool_changer.position.z = t.transform.translation.z;

  waypoints.push_back(at_tool_changer);

  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = floor_move_group_interface.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

  // Retime trajectory 
  robot_trajectory::RobotTrajectory rt(floor_move_group_interface.getCurrentState()->getRobotModel(), "floor_robot_whole");
  rt.setRobotTrajectoryMsg(*floor_move_group_interface.getCurrentState(), trajectory);
  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  totg.computeTimeStamps(rt, 0.05, 0.1);
  rt.getRobotTrajectoryMsg(trajectory);

  RCLCPP_INFO_STREAM(logger, "Path fraction: " << fraction);

  floor_move_group_interface.execute(trajectory);

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}