#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("moveit_test", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  moveit::planning_interface::MoveGroupInterface floor_move_group(node, "floor_robot");
  moveit::planning_interface::MoveGroupInterface ceiling_move_group(node, "ceiling_robot");

  floor_move_group.setMaxVelocityScalingFactor(1.0);
  floor_move_group.setMaxAccelerationScalingFactor(1.0);

  ceiling_move_group.setMaxVelocityScalingFactor(1.0);
  ceiling_move_group.setMaxAccelerationScalingFactor(1.0);

  floor_move_group.setNamedTarget("home");
  
  // Create a plan to that target pose
  auto const [success1, plan1] = [&floor_move_group]{
    moveit::planning_interface::MoveGroupInterface::Plan msg1;
    auto const ok1 = static_cast<bool>(floor_move_group.plan(msg1));
    return std::make_pair(ok1, msg1);
  }();

  // Execute the plan
  if(success1) {
    floor_move_group.execute(plan1);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Planing failed!");
  }

  ceiling_move_group.setNamedTarget("home");

  // Create a plan to that target pose
  auto const [success2, plan2] = [&ceiling_move_group]{
    moveit::planning_interface::MoveGroupInterface::Plan msg2;
    auto const ok2 = static_cast<bool>(ceiling_move_group.plan(msg2));
    return std::make_pair(ok2, msg2);
  }();

  // Execute the plan
  if(success2) {
    ceiling_move_group.execute(plan2);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Planing failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}