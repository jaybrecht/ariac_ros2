#include <test_competitor/test_competitor.hpp>

TestCompetitor::TestCompetitor()
 : Node("test_competitor"),
  floor_robot_(std::shared_ptr<rclcpp::Node>(std::move(this)), "floor_robot"),
  ceiling_robot_(std::shared_ptr<rclcpp::Node>(std::move(this)), "ceiling_robot"),
  // floor_arm_(std::shared_ptr<rclcpp::Node>(std::move(this)), "floor_arm"),
  // ceiling_arm_(std::shared_ptr<rclcpp::Node>(std::move(this)), "ceiling_arm"),
  // gantry_(std::shared_ptr<rclcpp::Node>(std::move(this)), "gantry"),
  planning_scene_()
{
  // Use upper joint velocity and acceleration limits
  floor_robot_.setMaxAccelerationScalingFactor(1.0);
  floor_robot_.setMaxVelocityScalingFactor(1.0);

  ceiling_robot_.setMaxAccelerationScalingFactor(1.0);
  ceiling_robot_.setMaxVelocityScalingFactor(1.0);

  // Subscribe to Logical Cameras 
  rclcpp::SubscriptionOptions options;

  topic_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  options.callback_group = topic_cb_group_;

  kts1_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
    "/ariac/sensors/kts1_camera/image", rclcpp::SensorDataQoS(), 
    std::bind(&TestCompetitor::kts1_camera_cb, this, std::placeholders::_1), options);

  kts2_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
    "/ariac/sensors/kts2_camera/image", rclcpp::SensorDataQoS(), 
    std::bind(&TestCompetitor::kts2_camera_cb, this, std::placeholders::_1), options);

  left_bins_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
    "/ariac/sensors/left_bins_camera/image", rclcpp::SensorDataQoS(), 
    std::bind(&TestCompetitor::left_bins_camera_cb, this, std::placeholders::_1), options);

  right_bins_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
    "/ariac/sensors/right_bins_camera/image", rclcpp::SensorDataQoS(), 
    std::bind(&TestCompetitor::right_bins_camera_cb, this, std::placeholders::_1), options);

  floor_gripper_state_sub_ = this->create_subscription<ariac_msgs::msg::VacuumGripperState>(
    "/ariac/floor_robot_gripper_state", rclcpp::SensorDataQoS(), 
    std::bind(&TestCompetitor::floor_gripper_state_cb, this, std::placeholders::_1), options);

  // Initialize service clients 
  floor_robot_tool_changer_ = this->create_client<ariac_msgs::srv::ChangeGripper>("/ariac/floor_robot_change_gripper");
  floor_robot_gripper_enable_ = this->create_client<ariac_msgs::srv::VacuumGripperControl>("/ariac/floor_robot_enable_gripper");

  RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}

TestCompetitor::~TestCompetitor() 
{
  floor_robot_.~MoveGroupInterface();
  ceiling_robot_.~MoveGroupInterface();
}

void TestCompetitor::kts1_camera_cb(
  const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg) 
{
  if (!kts1_camera_recieved_data) {
    RCLCPP_INFO(get_logger(), "Received data from kts1 camera");
    kts1_camera_recieved_data = true;
  }
  
  kts1_trays_ = msg->tray_poses;
  kts1_camera_pose_ = msg->sensor_pose;
}

void TestCompetitor::kts2_camera_cb(
  const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg) 
{
  if (!kts2_camera_recieved_data) {
    RCLCPP_INFO(get_logger(), "Received data from kts2 camera");
    kts2_camera_recieved_data = true;
  }

  kts2_trays_ = msg->tray_poses;
  kts2_camera_pose_ = msg->sensor_pose;
}

void TestCompetitor::left_bins_camera_cb(
  const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg) 
{
  if (!left_bins_camera_recieved_data) {
    RCLCPP_INFO(get_logger(), "Received data from left bins camera");
    left_bins_camera_recieved_data = true;
  }

  left_bins_parts_ = msg->part_poses;
  left_bins_camera_pose_ = msg->sensor_pose;
}

void TestCompetitor::right_bins_camera_cb(
  const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg) 
{
  if (!right_bins_camera_recieved_data) {
    RCLCPP_INFO(get_logger(), "Received data from right bins camera");
    right_bins_camera_recieved_data = true;
  }

  right_bins_parts_ = msg->part_poses;
  right_bins_camera_pose_ = msg->sensor_pose;
}

void TestCompetitor::floor_gripper_state_cb(
  const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg) 
{
  floor_gripper_state_ = *msg;
}

std::string TestCompetitor::LocateKitTrayStation(int tray_id){
  // Find which kit tray station has the requested tray id 
  //  either "kts1" or "kts2"
  //  returns an empty string if the requested id is not on either table
  
  for (auto tray: kts1_trays_) {
    if (tray.id == tray_id) {
      return "kts1";
    }
  }

  for (auto tray: kts2_trays_) {
    if (tray.id == tray_id) {
      return "kts2";
    }
  }

  return "";
}

bool TestCompetitor::MoveToKitTrayStation(std::string station){
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success;

  // Move to station 
  if (station == "kts1") {
    floor_robot_.setJointValueTarget("linear_actuator_joint", 4.5);
    floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 1.5);
  } else if (station == "kts2") {
    floor_robot_.setJointValueTarget("linear_actuator_joint", -4.5);
    floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", -1.5);
  }

  success = static_cast<bool>(floor_robot_.plan(plan));

  if (success) {
    floor_robot_.execute(plan);
  } else {
    RCLCPP_ERROR(get_logger(), "Unable to generate plan");
    return false;
  }

  return true;

}

geometry_msgs::msg::Pose TestCompetitor::LocateKitTray(std::string station, int tray_id)
{
  geometry_msgs::msg::Pose tray_rel_pose;
  geometry_msgs::msg::Pose sensor_pose;

  if (station == "kts1") {
    for (auto tray: kts1_trays_) {
      if (tray.id == tray_id) {
        tray_rel_pose = tray.pose;
        sensor_pose = kts1_camera_pose_;
      }
    }
  } else if (station == "kts2") {

    for (auto tray: kts2_trays_) {
      if (tray.id == tray_id) {
        tray_rel_pose = tray.pose;
        sensor_pose = kts1_camera_pose_;
      }
    }
  } else {
    RCLCPP_ERROR(get_logger(), "Unable to locate tray id at requested station");
    geometry_msgs::msg::Pose empty_pose;

    return empty_pose;
  }

  KDL::Frame tray_in_camera_frame;
  KDL::Frame camera_in_world_frame;

  tf2::fromMsg(tray_rel_pose, tray_in_camera_frame);
  tf2::fromMsg(sensor_pose, camera_in_world_frame);

  KDL::Frame tray_in_world_frame = camera_in_world_frame*tray_in_camera_frame;
  
  return tf2::toMsg(tray_in_world_frame);
}

geometry_msgs::msg::Pose TestCompetitor::LocateBinPart(ariac_msgs::msg::Part part)
{
  geometry_msgs::msg::Pose part_rel_pose;
  geometry_msgs::msg::Pose sensor_pose;

  bool found_part = false;

  // Create list of all bin parts
  std::vector<ariac_msgs::msg::PartPose> bin_parts;
  for (auto bin_part: left_bins_parts_) {
    bin_parts.push_back(bin_part);
  }

  for (auto bin_part: right_bins_parts_) {
    bin_parts.push_back(bin_part);
  }

  for (auto bin_part: bin_parts) {
    if (bin_part.part.type == part.type && bin_part.part.color == part.color) {
      part_rel_pose = bin_part.pose;
      sensor_pose = left_bins_camera_pose_;
      found_part = true;
    }
  }

  if (!found_part) {
    RCLCPP_ERROR(get_logger(), "Unable to locate tray id at requested station");
    geometry_msgs::msg::Pose empty_pose;

    return empty_pose;
  }  

  KDL::Frame part_in_camera_frame;
  KDL::Frame camera_in_world_frame;

  tf2::fromMsg(part_rel_pose, part_in_camera_frame);
  tf2::fromMsg(sensor_pose, camera_in_world_frame);

  KDL::Frame part_in_world_frame = camera_in_world_frame*part_in_camera_frame;
  
  return tf2::toMsg(part_in_world_frame);
}

bool TestCompetitor::ChangeFloorRobotTool(std::string station, std::string gripper_type){
  // Lookup tf frame for gripper station 
  geometry_msgs::msg::TransformStamped t;

  std::string frame_id = station + "_tool_changer_" + gripper_type + "_frame";

  try {
    t = tf_buffer->lookupTransform("world", frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(get_logger(), "Could not get transform");
    return false;
  }
  std::vector<geometry_msgs::msg::Pose> waypoints;
  
  geometry_msgs::msg::Pose above_tool_changer;
  above_tool_changer = floor_robot_.getCurrentPose().pose;
  above_tool_changer.position.x = t.transform.translation.x;
  above_tool_changer.position.y = t.transform.translation.y;

  waypoints.push_back(above_tool_changer);

  geometry_msgs::msg::Pose at_tool_changer;
  at_tool_changer = floor_robot_.getCurrentPose().pose;
  at_tool_changer.position.x = t.transform.translation.x;
  at_tool_changer.position.y = t.transform.translation.y;
  at_tool_changer.position.z = t.transform.translation.z;

  waypoints.push_back(at_tool_changer);

  moveit_msgs::msg::RobotTrajectory trajectory;
  floor_robot_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

  // Retime trajectory 
  robot_trajectory::RobotTrajectory rt(floor_robot_.getCurrentState()->getRobotModel(), "floor_robot");
  rt.setRobotTrajectoryMsg(*floor_robot_.getCurrentState(), trajectory);
  totg_.computeTimeStamps(rt, 0.5, 0.3);
  rt.getRobotTrajectoryMsg(trajectory);

  floor_robot_.execute(trajectory);

  auto request = std::make_shared<ariac_msgs::srv::ChangeGripper::Request>();
  
  if (gripper_type == "trays") {
    request->gripper_type = ariac_msgs::srv::ChangeGripper::Request::TRAY_GRIPPER;
  } else if (gripper_type == "parts") {
    request->gripper_type = ariac_msgs::srv::ChangeGripper::Request::PART_GRIPPER;
  }

  auto result =floor_robot_tool_changer_->async_send_request(request);

  result.wait();

  waypoints.clear();
  at_tool_changer.position.z += 0.3;
  waypoints.push_back(at_tool_changer);

  floor_robot_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

  // Retime trajectory 
  rt.setRobotTrajectoryMsg(*floor_robot_.getCurrentState(), trajectory);
  totg_.computeTimeStamps(rt, 0.5, 0.3);
  rt.getRobotTrajectoryMsg(trajectory);

  floor_robot_.execute(trajectory);

  if (result.get()->success) {
    return true;
  } else {
    RCLCPP_ERROR_STREAM(get_logger(), result.get()->message);
    return false;
  }
}

bool TestCompetitor::PickTray(int tray_id)
{
  std::string station = LocateKitTrayStation(tray_id);
	
  if (station == "")
		return false;

  MoveToKitTrayStation(station);

  if (floor_gripper_state_.type != "tray_gripper") {
    ChangeFloorRobotTool(station, "trays");
  }

	geometry_msgs::msg::Pose tray_pose = LocateKitTray(station, tray_id);
  
  std::vector<geometry_msgs::msg::Pose> waypoints;
  
  geometry_msgs::msg::Pose above_tray;
  above_tray = floor_robot_.getCurrentPose().pose;
  above_tray.position.x = tray_pose.position.x;
  above_tray.position.y = tray_pose.position.y;
  above_tray.position.z = tray_pose.position.z + 0.2;

  waypoints.push_back(above_tray);

  geometry_msgs::msg::Pose tray_grip;
  tray_grip = floor_robot_.getCurrentPose().pose;
  tray_grip.position.x = tray_pose.position.x;
  tray_grip.position.y = tray_pose.position.y;
  tray_grip.position.z = tray_pose.position.z - 0.001;

  waypoints.push_back(tray_grip);

  moveit_msgs::msg::RobotTrajectory trajectory;
  floor_robot_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

  robot_trajectory::RobotTrajectory rt(floor_robot_.getCurrentState()->getRobotModel(), "floor_robot");
  rt.setRobotTrajectoryMsg(*floor_robot_.getCurrentState(), trajectory);
  totg_.computeTimeStamps(rt, 0.2, 0.1);
  rt.getRobotTrajectoryMsg(trajectory);

  floor_robot_.execute(trajectory);

  auto request = std::make_shared<ariac_msgs::srv::VacuumGripperControl::Request>();
  request->enable = true;

  auto result =floor_robot_gripper_enable_->async_send_request(request);
  result.wait();

  while (!floor_gripper_state_.attached) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");
  }

  waypoints.clear();

  waypoints.push_back(above_tray);

  floor_robot_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

  // Retime trajectory 
  rt.setRobotTrajectoryMsg(*floor_robot_.getCurrentState(), trajectory);
  totg_.computeTimeStamps(rt, 0.2, 0.3);
  rt.getRobotTrajectoryMsg(trajectory);

  floor_robot_.execute(trajectory);

  return false;
}

bool TestCompetitor::PickBinPart(ariac_msgs::msg::Part part_to_pick)
{
	geometry_msgs::msg::Pose part_pose = LocateBinPart(part_to_pick);
  
  std::vector<geometry_msgs::msg::Pose> waypoints;
  
  geometry_msgs::msg::Pose above_part;
  above_part = floor_robot_.getCurrentPose().pose;
  above_part.position.x = part_pose.position.x;
  above_part.position.y = part_pose.position.y;
  above_part.position.z = part_pose.position.z + 0.4;

  waypoints.push_back(above_part);

  std::map<int, double> part_heights;
  part_heights.insert({ariac_msgs::msg::Part::BATTERY, 0.04});
  part_heights.insert({ariac_msgs::msg::Part::PUMP, 0.12});
  part_heights.insert({ariac_msgs::msg::Part::REGULATOR, 0.07});
  part_heights.insert({ariac_msgs::msg::Part::SENSOR, 0.10});

  geometry_msgs::msg::Pose part_grip;
  part_grip = floor_robot_.getCurrentPose().pose;
  part_grip.position.x = part_pose.position.x;
  part_grip.position.y = part_pose.position.y;
  part_grip.position.z = part_pose.position.z + part_heights[part_to_pick.type] - 0.001;

  waypoints.push_back(part_grip);

  moveit_msgs::msg::RobotTrajectory trajectory;
  floor_robot_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

  robot_trajectory::RobotTrajectory rt(floor_robot_.getCurrentState()->getRobotModel(), "floor_robot");
  rt.setRobotTrajectoryMsg(*floor_robot_.getCurrentState(), trajectory);
  totg_.computeTimeStamps(rt, 0.2, 0.1);
  rt.getRobotTrajectoryMsg(trajectory);

  floor_robot_.execute(trajectory);

  auto request = std::make_shared<ariac_msgs::srv::VacuumGripperControl::Request>();
  request->enable = true;

  auto result =floor_robot_gripper_enable_->async_send_request(request);
  result.wait();

  while (!floor_gripper_state_.attached) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");
  }

  waypoints.clear();

  waypoints.push_back(above_part);

  floor_robot_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

  // Retime trajectory 
  rt.setRobotTrajectoryMsg(*floor_robot_.getCurrentState(), trajectory);
  totg_.computeTimeStamps(rt, 0.2, 0.3);
  rt.getRobotTrajectoryMsg(trajectory);

  floor_robot_.execute(trajectory);

  return false;
}

bool TestCompetitor::PlaceTrayOnAGV(int agv_num)
{
  geometry_msgs::msg::TransformStamped t;

  std::string frame_id = "agv" + std::to_string(agv_num) + "_tray";

  try {
    t = tf_buffer->lookupTransform("world", frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(get_logger(), "Could not get transform");
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  double linear_joint_pos;
  switch(agv_num) {
    case 1 :
      linear_joint_pos = -4.5;
      break;
    case 2 :
      linear_joint_pos = -1.2;
      break;
    case 3 :
      linear_joint_pos = 1.2;
      break;
    case 4 :
      linear_joint_pos = 4.5;
      break;
    default :
      linear_joint_pos = 0.0;
      break;   
  }

  floor_robot_.setJointValueTarget("linear_actuator_joint", linear_joint_pos);
  floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);

  bool success = static_cast<bool>(floor_robot_.plan(plan));

  if (success) {
    floor_robot_.execute(plan);
  } else {
    RCLCPP_ERROR(get_logger(), "Unable to generate plan");
    return false;
  }

  std::vector<geometry_msgs::msg::Pose> waypoints;
  
  geometry_msgs::msg::Pose above_agv;
  above_agv = floor_robot_.getCurrentPose().pose;
  above_agv.position.x = t.transform.translation.x;
  above_agv.position.y = t.transform.translation.y;

  waypoints.push_back(above_agv);

  moveit_msgs::msg::RobotTrajectory trajectory;
  floor_robot_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

  // Retime trajectory 
  robot_trajectory::RobotTrajectory rt(floor_robot_.getCurrentState()->getRobotModel(), "floor_robot");
  rt.setRobotTrajectoryMsg(*floor_robot_.getCurrentState(), trajectory);
  totg_.computeTimeStamps(rt, 0.5, 0.3);
  rt.getRobotTrajectoryMsg(trajectory);

  floor_robot_.execute(trajectory);

  return true;
}

bool TestCompetitor::CompleteKittingTask(ariac_msgs::msg::KittingTask &task) {
	// Find which kit tray station has the desired tray
	PickTray(task.tray_id);

	// // Place the picked tray onto the 
	PlaceTrayOnAGV(task.agv_number);
	
	// LockTrayOnAGV(task.agv_number);

	// // Switch to the part gripper at the requested station
	// ChangeFloorRobotTool(station, "parts");	

	// for (auto kitting_part: task.parts) {
	// 	// Locate the part in the list of bins
	// 	geometry_msgs::msg::Pose part_pose;
	// 	part_pose = LocateBinPart(kitting_part.part);
		
	// 	// Pick the part
	// 	PickBinPart(part_pose);
		
	// 	// Place the part on the kit tray
	// 	PlacePartOnKitTray(task.agv_number, kitting_part.quadrant);
	// }

	// // Check to see if the tray passes the quality check 
	// bool passed = PerformQualityCheck(task.agv_number);
	
	// if (passed) {
	// 	// Move the AGV to the requested location
	// 	MoveAGV(task.destination);
	// } 

  return true;
}

bool TestCompetitor::SendRobotToHome(std::string robot_name)
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool plan_success;
  bool motion_success;
  std::string target = "home";

  if (robot_name == "floor_robot") {
    floor_robot_.setNamedTarget(target);

    plan_success = static_cast<bool>(floor_robot_.plan(plan));

    if (plan_success) {
      motion_success = static_cast<bool>(floor_robot_.execute(plan));
      if (motion_success) {
        floor_robot_home_orientation_ = floor_robot_.getCurrentPose().pose.orientation;
        return true;
      } 
    }
  } else if (robot_name == "ceiling_robot") {
    
    ceiling_robot_.setNamedTarget(target);

    plan_success = static_cast<bool>(ceiling_robot_.plan(plan));

    if (plan_success) {
      motion_success = static_cast<bool>(ceiling_robot_.execute(plan));
      if (motion_success) {
        ceiling_robot_home_orientation_ = ceiling_robot_.getCurrentPose().pose.orientation;
        return true;
      }
    }
  } else {
    RCLCPP_ERROR(get_logger(), "Not a valid robot name");
    return false;
  }

  if (!plan_success) {
    RCLCPP_ERROR(get_logger(), "Unable to execute plan");
  } else if (!motion_success) {
    RCLCPP_ERROR(get_logger(), "Unable to execute plan");
  }

  return false;
}