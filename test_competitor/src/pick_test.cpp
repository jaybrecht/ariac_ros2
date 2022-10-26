#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.h>
#include <ariac_msgs/msg/logical_camera_image.hpp>
#include <ariac_msgs/msg/model.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <kdl/frames.hpp>
#include <tf2_kdl/tf2_kdl.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sstream>
#include <filesystem>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

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
  // Create planning scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;


  /// Method to determine if part exists in bin
  bool part_exists(std::string type, std::string color);
  /// Method to determine pose of part in world form
  geometry_msgs::msg::Pose get_pose_of_model();
  /// Add model to robots planning scene
  void add_model_to_planning_scene(std::string name, std::string model_type, geometry_msgs::msg::Pose model_pose);

private:
  /// Callback that plans and executes trajectory each time the target pose is changed
  void logical_camera_callback(const ariac_msgs::msg::LogicalCameraImage::ConstSharedPtr msg);
};

FloorRobotCommander::FloorRobotCommander(moveit::planning_interface::MoveGroupInterface::Options options) : Node("floor_robot_commander"),
  move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), options),
  planning_scene_interface_()
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

void FloorRobotCommander::add_model_to_planning_scene(std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose){
  moveit_msgs::msg::CollisionObject collision;

  collision.id = name;
  collision.header.frame_id = "world";

  shape_msgs::msg::Mesh mesh;
  shapes::ShapeMsg mesh_msg;
  
  std::string package_share_directory = ament_index_cpp::get_package_share_directory("test_competitor");
  std::stringstream path;
  path << "file://" << package_share_directory << "/meshes/" << mesh_file;
  std::string model_path = path.str();

  // RCLCPP_INFO(this->get_logger(), model_path.c_str());

  //Check if file exists
  // if (!std::filesystem::exists(model_path)){
  //   RCLCPP_WARN(this->get_logger(), "Unable to locate file");
  //   return;
  // }

  shapes::Mesh *m = shapes::createMeshFromResource(model_path);
  shapes::constructMsgFromShape(m, mesh_msg);

  mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

  collision.meshes.push_back(mesh);
  collision.mesh_poses.push_back(model_pose);

  collision.operation = collision.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision);

  planning_scene_interface_.addCollisionObjects(collision_objects);
}

geometry_msgs::msg::Quaternion quaternion_from_rpy(double r, double p, double y){
  tf2::Quaternion q;
  geometry_msgs::msg::Quaternion q_msg;

  q.setRPY(r, p, y);

  q_msg.x = q.x();
  q_msg.y = q.y();
  q_msg.z = q.z();
  q_msg.w = q.w();

  return q_msg;
}


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  moveit::planning_interface::MoveGroupInterface::Options options("floor_robot_whole", "robot_description", "/floor_robot");

  auto floor_robot_commander = std::make_shared<FloorRobotCommander>(options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(floor_robot_commander);
  std::thread([&executor]() { executor.spin(); }).detach();

  geometry_msgs::msg::Pose bin1_pose;
  bin1_pose.position.x = -1.898993;
  bin1_pose.position.y = 3.379920;
  bin1_pose.position.z = 0;
  bin1_pose.orientation = quaternion_from_rpy(0, 0, 3.14159);

  floor_robot_commander->add_model_to_planning_scene("bin1", "bin.stl", bin1_pose);

  geometry_msgs::msg::Pose bin2_pose;
  bin2_pose.position.x = -1.898993;
  bin2_pose.position.y = 2.565006;
  bin2_pose.position.z = 0;
  bin2_pose.orientation = quaternion_from_rpy(0, 0, 3.14159);

  floor_robot_commander->add_model_to_planning_scene("bin2", "bin.stl", bin2_pose);

  geometry_msgs::msg::Pose bin3_pose;
  bin3_pose.position.x = -2.651690;
  bin3_pose.position.y = 2.565006;
  bin3_pose.position.z = 0;
  bin3_pose.orientation = quaternion_from_rpy(0, 0, 3.14159);

  floor_robot_commander->add_model_to_planning_scene("bin3", "bin.stl", bin3_pose);

  geometry_msgs::msg::Pose bin4_pose;
  bin4_pose.position.x = -2.651690;
  bin4_pose.position.y = 3.379920;
  bin4_pose.position.z = 0;
  bin4_pose.orientation = quaternion_from_rpy(0, 0, 3.14159);

  floor_robot_commander->add_model_to_planning_scene("bin4", "bin.stl", bin4_pose);

  geometry_msgs::msg::Pose conveyor_pose;
  conveyor_pose.position.x = 0.573076;
  conveyor_pose.position.y = 0;
  conveyor_pose.position.z = 0;
  conveyor_pose.orientation = quaternion_from_rpy(0, 0, 0);

  floor_robot_commander->add_model_to_planning_scene("conveyor", "conveyor.dae", conveyor_pose);

  geometry_msgs::msg::Pose floor_pose;
  floor_pose.position.x = -3.6;
  floor_pose.position.y = -3.32;
  floor_pose.position.z = 0;
  floor_pose.orientation = quaternion_from_rpy(0, 0, 0);

  floor_robot_commander->add_model_to_planning_scene("floor", "floor.dae", floor_pose);

  // Move robot to home position
  floor_robot_commander->move_group_.setNamedTarget("home");
  floor_robot_commander->move_group_.move();

  // Move robot above battery
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "world";
  while (rclcpp::ok()){
    if (floor_robot_commander->part_exists("battery", "blue")){
      pose.pose = floor_robot_commander->get_pose_of_model();
      break;
    }
  }

  // Set orientation of gripper
  auto rpy = floor_robot_commander->move_group_.getCurrentRPY();
  pose.pose.orientation = quaternion_from_rpy(rpy[0], rpy[1], rpy[2]);
  pose.pose.position.z += 0.2;

  floor_robot_commander->move_group_.setPoseTarget(pose);
  floor_robot_commander->move_group_.move();

  pose.pose.position.z -= 0.164;

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(pose.pose);

  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = floor_robot_commander->move_group_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

  // Retime trajectory 
  robot_trajectory::RobotTrajectory rt(floor_robot_commander->move_group_.getCurrentState()->getRobotModel(), "floor_robot_whole");
  rt.setRobotTrajectoryMsg(*floor_robot_commander->move_group_.getCurrentState(), trajectory);
  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  totg.computeTimeStamps(rt, 0.05, 0.1);
  rt.getRobotTrajectoryMsg(trajectory);

  RCLCPP_INFO_STREAM(floor_robot_commander->get_logger(), "Path fraction: " << fraction);

  floor_robot_commander->move_group_.execute(trajectory);

  pose.pose.position.z += 0.164;

  waypoints.clear();
  waypoints.push_back(pose.pose);

  moveit_msgs::msg::RobotTrajectory trajectory2;
  fraction = floor_robot_commander->move_group_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory2);

  // Retime trajectory 
  rt.setRobotTrajectoryMsg(*floor_robot_commander->move_group_.getCurrentState(), trajectory2);
  totg.computeTimeStamps(rt, 0.05, 0.1);
  rt.getRobotTrajectoryMsg(trajectory2);

  RCLCPP_INFO_STREAM(floor_robot_commander->get_logger(), "Path fraction: " << fraction);

  floor_robot_commander->move_group_.execute(trajectory2);

  rclcpp::shutdown();
}