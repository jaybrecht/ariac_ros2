#include <memory>
#include <string> 

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>

class MinimalMoveIt : public rclcpp::Node
{
  public:

};


int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("moveit_test", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("moveit_test");

  std::string group = "kitting_arm";
  std::string desc = "floor/robot_description";
  std::string move_group_namespace = "kitting";

  moveit::planning_interface::MoveGroupInterface::Options opt1(group, desc, move_group_namespace);

  moveit::planning_interface::MoveGroupInterface floor_move_group_interface(node, opt1);

  group = "assembly_arm";
  desc = "ceiling/robot_description";
  move_group_namespace = "assembly";

  moveit::planning_interface::MoveGroupInterface::Options opt2(group, desc, move_group_namespace);

  moveit::planning_interface::MoveGroupInterface cieling_move_group_interface(node, opt2);


  cieling_move_group_interface.setNamedTarget("assembly_test_configuration");

  moveit::planning_interface::MoveGroupInterface::Plan plan1;
  if (cieling_move_group_interface.plan(plan1) == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    cieling_move_group_interface.execute(plan1);
  }
  else {
    RCLCPP_WARN(logger, "Unable to generate plan to home");
  }

  floor_move_group_interface.setNamedTarget("kitting_test_configuration");

  moveit::planning_interface::MoveGroupInterface::Plan plan2;
  if (floor_move_group_interface.plan(plan2) == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    floor_move_group_interface.execute(plan2); 
  }
  else {
    RCLCPP_WARN(logger, "Unable to generate plan to home");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}