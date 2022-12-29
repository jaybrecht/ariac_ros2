#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "floor_robot_moveit_test");

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  moveit::planning_interface::MoveGroupInterface::Options floor_opt(
    "floor_robot_whole", "floor_robot_description", "/floor_robot");

  moveit::planning_interface::MoveGroupInterface floor_move_group_interface(node, floor_opt);

  moveit::planning_interface::MoveGroupInterface::Options ceiling_opt(
    "ceiling_robot_whole", "ceiling_robot_description", "/ceiling_robot");

  moveit::planning_interface::MoveGroupInterface ceiling_move_group_interface(node, ceiling_opt);

  floor_move_group_interface.setNamedTarget("home");
  
  // Create a plan to that target pose
  auto const [success1, plan1] = [&floor_move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg1;
    auto const ok1 = static_cast<bool>(floor_move_group_interface.plan(msg1));
    return std::make_pair(ok1, msg1);
  }();

  // Execute the plan
  if(success1) {
    floor_move_group_interface.execute(plan1);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  ceiling_move_group_interface.setNamedTarget("home");

  // Create a plan to that target pose
  auto const [success2, plan2] = [&ceiling_move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg2;
    auto const ok2 = static_cast<bool>(ceiling_move_group_interface.plan(msg2));
    return std::make_pair(ok2, msg2);
  }();

  // Execute the plan
  if(success2) {
    ceiling_move_group_interface.execute(plan2);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  rclcpp::spin(node);

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}