#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <kdl/frames.hpp>
#include <tf2_kdl/tf2_kdl.h>

#include <ariac_msgs/msg/advanced_logical_camera_image.hpp>
#include <ariac_msgs/msg/kitting_task.hpp>
#include <ariac_msgs/msg/kit_tray_pose.hpp>
#include <ariac_msgs/msg/vacuum_gripper_state.hpp>

#include <ariac_msgs/srv/change_gripper.hpp>
#include <ariac_msgs/srv/vacuum_gripper_control.hpp>

#include <geometry_msgs/msg/pose.hpp>

class TestCompetitor : public rclcpp::Node
{
public:
  /// Constructor
  TestCompetitor();

  ~TestCompetitor();

  // Floor Robot Public Functions
  bool FloorRobotSendHome();
  bool FloorRobotChangeGripper(std::string gripper_type);
  bool FloorRobotPickTray(int tray_id);
  bool FloorRobotPickBinPart(ariac_msgs::msg::Part part_to_pick);
  bool FloorRobotPickConveyorPart(ariac_msgs::msg::Part part_to_pick);
  bool FloorRobotPlaceTrayOnAGV(std::string agv);
  bool FloorRobotPlacePartOnKitTray(std::string agv, int quadrant);

  // ARIAC Functions
  bool LockTrayOnAGV(std::string agv);
  bool PerformQualityCheck(int);
  bool MoveAGV(int destination);

  // Analyze Data from Sensors
  std::string LocateKitTrayStation(int);
  geometry_msgs::msg::Pose LocateKitTray(std::string, int);
  geometry_msgs::msg::Pose LocateBinPart(ariac_msgs::msg::Part);

  bool ChangeFloorRobotTool(std::string, std::string);
  bool MoveToKitTrayStation(std::string);
  bool LockTrayOnAGV(int);
  bool PickTray(int);
  bool PickBinPart(ariac_msgs::msg::Part);
  bool PlaceTrayOnAGV(int);
  bool PlacePartOnKitTray(int, int);
  bool CompleteKittingTask(ariac_msgs::msg::KittingTask &task);
  bool SendRobotToHome(std::string);

private:
  // Callback Groups
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  rclcpp::CallbackGroup::SharedPtr topic_cb_group_;

  // MoveIt Interfaces 
  moveit::planning_interface::MoveGroupInterface floor_robot_;
  moveit::planning_interface::MoveGroupInterface ceiling_robot_;
  
  moveit::planning_interface::PlanningSceneInterface planning_scene_;

  geometry_msgs::msg::Quaternion floor_robot_home_orientation_;
  geometry_msgs::msg::Quaternion ceiling_robot_home_orientation_;

  trajectory_processing::TimeOptimalTrajectoryGeneration totg_;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
  std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // Subscriptions
  rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr kts1_camera_sub_;
  rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr kts2_camera_sub_;
  rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr left_bins_camera_sub_;
  rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr right_bins_camera_sub_;

  rclcpp::Subscription<ariac_msgs::msg::VacuumGripperState>::SharedPtr floor_gripper_state_sub_;

  // Gripper State
  ariac_msgs::msg::VacuumGripperState floor_gripper_state_;

  // Sensor poses
  geometry_msgs::msg::Pose kts1_camera_pose_;
  geometry_msgs::msg::Pose kts2_camera_pose_;
  geometry_msgs::msg::Pose left_bins_camera_pose_;
  geometry_msgs::msg::Pose right_bins_camera_pose_;

  // Trays
  std::vector<ariac_msgs::msg::KitTrayPose> kts1_trays_;
  std::vector<ariac_msgs::msg::KitTrayPose> kts2_trays_;

  // Bins
  std::vector<ariac_msgs::msg::PartPose> left_bins_parts_;
  std::vector<ariac_msgs::msg::PartPose> right_bins_parts_;

  // Sensor Callbacks
  bool kts1_camera_recieved_data = false;
  bool kts2_camera_recieved_data = false;
  bool left_bins_camera_recieved_data = false;
  bool right_bins_camera_recieved_data = false;

  void kts1_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
  void kts2_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
  void left_bins_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
  void right_bins_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);

  // Gripper State Callback
  void floor_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg);

  // ARIAC Services 
  rclcpp::Client<ariac_msgs::srv::ChangeGripper>::SharedPtr floor_robot_tool_changer_;
  rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr floor_robot_gripper_enable_;
};