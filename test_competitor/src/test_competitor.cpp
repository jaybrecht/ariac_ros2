#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include <ariac_msgs/msg/advanced_logical_camera_image.hpp>


class FloorRobotCommander : public rclcpp::Node
{
public:
  /// Constructor
  FloorRobotCommander();

  /// Move group interface for the robot
  moveit::planning_interface::MoveGroupInterface::Options options_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  
  // Create planning scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
};

FloorRobotCommander::FloorRobotCommander()
 : Node("test_competitor"),
  options_("floor_robot_whole", "floor_robot_description", "/floor_robot"),
  move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), options_),
  planning_scene_interface_()
{
  // Use upper joint velocity and acceleration limits
  move_group_.setMaxAccelerationScalingFactor(1.0);
  move_group_.setMaxVelocityScalingFactor(1.0);

  RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto floor_robot_commander = std::make_shared<FloorRobotCommander>();

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  executor->add_node(floor_robot_commander);
  std::thread([&executor]() { executor->spin(); }).detach();

  auto pose = floor_robot_commander->move_group_.getCurrentPose().pose;

  RCLCPP_INFO_STREAM(floor_robot_commander->get_logger(), "Gripper Position: (x: " << pose.position.x 
    << ", y: " << pose.position.y << ", z: " << pose.position.z << ")");

  rclcpp::shutdown();
}