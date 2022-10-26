#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ariac_msgs/msg/logical_camera_image.hpp>
#include <ariac_msgs/msg/model.hpp>
#include <kdl/frames.hpp>
#include <tf2_kdl/tf2_kdl.h>

class FloorRobotCommander : public rclcpp::Node
{
public:
  /// Constructor
  FloorRobotCommander(moveit::planning_interface::MoveGroupInterface::Options options);

  /// Move group interface for the robot
  moveit::planning_interface::MoveGroupInterface move_group_;
  /// Subscriber for logical camera image
  rclcpp::Subscription<ariac_msgs::msg::LogicalCameraImage>::SharedPtr logical_camera_sub_;
  /// List of models in bins
  std::vector<ariac_msgs::msg::Model> models_in_bins_;
  /// Camera pose
  geometry_msgs::msg::Pose camera_pose_;
  geometry_msgs::msg::Pose model_pose_;

  /// Method to determine if part exists in bin
  bool part_exists(std::string type, std::string color);
  /// Method to determine pose of part in world form
  geometry_msgs::msg::Pose get_pose_of_model();

private:
  /// Callback that plans and executes trajectory each time the target pose is changed
  void logical_camera_callback(const ariac_msgs::msg::LogicalCameraImage::ConstSharedPtr msg);
};

FloorRobotCommander::FloorRobotCommander(moveit::planning_interface::MoveGroupInterface::Options options) : Node("floor_robot_commander"),
  move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), options)
{
  // Use upper joint velocity and acceleration limits
  this->move_group_.setMaxAccelerationScalingFactor(1.0);
  this->move_group_.setMaxVelocityScalingFactor(1.0);

  // Subscribe to target pose
  logical_camera_sub_ = this->create_subscription<ariac_msgs::msg::LogicalCameraImage>(
    "/ariac/right_bins_logical_camera", rclcpp::SensorDataQoS(), std::bind(&FloorRobotCommander::logical_camera_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}

void FloorRobotCommander::logical_camera_callback(const ariac_msgs::msg::LogicalCameraImage::ConstSharedPtr msg)
{
  // RCLCPP_INFO(this->get_logger(), "recieved message");
  camera_pose_ = msg->pose;
  models_in_bins_.clear();
  for (auto &model : msg->models){
    models_in_bins_.push_back(model);
  }
}

bool FloorRobotCommander::part_exists(std::string type, std::string color){
  for (auto &model: models_in_bins_){
    if (type == model.type && color == model.color){
      model_pose_ = model.pose;
      return true;
    }
  }
  return false;

}

geometry_msgs::msg::Pose FloorRobotCommander::get_pose_of_model(){
  KDL::Frame model_in_camera_frame;
  KDL::Frame camera_in_world_frame;

  tf2::fromMsg(model_pose_, model_in_camera_frame);
  tf2::fromMsg(camera_pose_, camera_in_world_frame);

  KDL::Frame model_in_world_frame = camera_in_world_frame*model_in_camera_frame;
  
  return tf2::toMsg(model_in_world_frame);
}


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  moveit::planning_interface::MoveGroupInterface::Options options("floor_arm", "robot_description", "floor_robot");

  auto floor_robot_commander = std::make_shared<FloorRobotCommander>(options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(floor_robot_commander);
  std::thread([&executor]() { executor.spin(); }).detach();

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "world";
  while (rclcpp::ok()){
    if (floor_robot_commander->part_exists("battery", "blue")){
      pose.pose = floor_robot_commander->get_pose_of_model();
      break;
    }
  }
  
  // Set orientation of gripper
  tf2::Quaternion q;
  q.setRPY(0, 3.14159, 0);

  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();

  pose.pose.position.z += 0.2;

  floor_robot_commander->move_group_.setNamedTarget("floor_up");
  floor_robot_commander->move_group_.move();

  floor_robot_commander->move_group_.setPoseTarget(pose);
  floor_robot_commander->move_group_.move();

  rclcpp::shutdown();
}