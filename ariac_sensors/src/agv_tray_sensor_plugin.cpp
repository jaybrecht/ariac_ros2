#include <ariac_sensors/agv_tray_sensor_plugin.hpp>
#include <gazebo/sensors/LogicalCameraSensor.hh>

#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <ariac_msgs/srv/perform_quality_check.hpp>
#include <ariac_msgs/msg/quality_issue.hpp>
#include <ariac_msgs/msg/part.hpp>
#include <ariac_msgs/msg/part_pose.hpp>
#include <ariac_msgs/msg/kit_tray_pose.hpp>
#include <ariac_msgs/msg/order.hpp>
#include <ariac_msgs/msg/kitting_task.hpp>
#include <ariac_msgs/msg/kitting_part.hpp>
#include <ariac_msgs/msg/parts.hpp>

#include <tf2_kdl/tf2_kdl.h>
#include <tf2/convert.h>
#include <kdl/frames.hpp>

#include <memory>
#include <map>

namespace ariac_sensors
{

struct TrayParts{
  std::map<int, ariac_msgs::msg::PartPose> parts;
  std::map<int, std::string> part_names;
};

class AGVTraySensorPluginPrivate
{
public:
  /// Node for ros communication
  gazebo_ros::Node::SharedPtr ros_node_;
  /// LogicalCamera sensor this plugin is attached to
  gazebo::sensors::LogicalCameraSensorPtr sensor_;
  /// Event triggered when sensor updates
  gazebo::event::ConnectionPtr sensor_update_event_;

  std::string sensor_num_;

  std::map<std::string, int> part_types_;
  std::map<std::string, int> part_colors_;

  /// Lists of parts and trays
  std::map<std::string, ariac_msgs::msg::PartPose> detected_parts_;
  ariac_msgs::msg::KitTrayPose kit_tray_;

  /// List of orders
  std::vector<ariac_msgs::msg::Order> orders_;
  std::map<std::string, std::map<int, bool>> faulty_part_info_; 
  std::map<std::string, std::map<int, std::string>> faulty_part_names_; 

  geometry_msgs::msg::Pose sensor_pose_;

  rclcpp::Service<ariac_msgs::srv::PerformQualityCheck>::SharedPtr quality_check_service_;

  rclcpp::Publisher<ariac_msgs::msg::Parts>::SharedPtr tray_contents_pub_;
  ariac_msgs::msg::Parts tray_contents_msg_;

  /// Publish latest logical camera data to ROS
  void OnUpdate();

  // 
  void PerformQualityCheck(
    ariac_msgs::srv::PerformQualityCheck::Request::SharedPtr,
    ariac_msgs::srv::PerformQualityCheck::Response::SharedPtr
  );

  void FillOrders();

  TrayParts LocatePartsOnTray();
  
  bool CheckFlipped(
    ariac_msgs::msg::KitTrayPose tray_pose, 
    ariac_msgs::msg::PartPose part_pose);
};

AGVTraySensorPlugin::AGVTraySensorPlugin()
: impl_(std::make_unique<AGVTraySensorPluginPrivate>())
{
}

AGVTraySensorPlugin::~AGVTraySensorPlugin()
{
}

void AGVTraySensorPlugin::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  this->impl_->sensor_ = std::dynamic_pointer_cast<gazebo::sensors::LogicalCameraSensor>(_sensor);
  this->impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  
  if (!_sdf->HasElement("sensor_num")) {
    impl_->sensor_num_ = "0";
  } else {
    impl_->sensor_num_ = _sdf->Get<std::string>("sensor_num");
  }

  impl_->FillOrders();

  this->impl_->part_types_ = {{"battery", 10}, {"pump", 11}, {"sensor", 12}, {"regulator", 13}}; 
  this->impl_->part_colors_ = {{"red", 0}, {"green", 1}, {"blue", 2}, {"orange", 3}, {"purple", 4}};

  // Create quality control service 
  impl_->quality_check_service_ = this->impl_->ros_node_->create_service<ariac_msgs::srv::PerformQualityCheck>(
  "/ariac/perform_quality_check_agv" + impl_->sensor_num_, 
  std::bind(
    &AGVTraySensorPluginPrivate::PerformQualityCheck, this->impl_.get(),
    std::placeholders::_1, std::placeholders::_2));

  // Register publisher
  std::string tray_contents_topic = "/ariac/agv" + impl_->sensor_num_ + "_tray_contents";
  impl_->tray_contents_pub_ = impl_->ros_node_->create_publisher<ariac_msgs::msg::Parts>(tray_contents_topic, 10);

  impl_->sensor_update_event_ = impl_->sensor_->ConnectUpdated(
    std::bind(&AGVTraySensorPluginPrivate::OnUpdate, this->impl_.get()));
}

void AGVTraySensorPluginPrivate::OnUpdate()
{
  const auto & image = sensor_->Image();

  sensor_pose_ = gazebo_ros::Convert<geometry_msgs::msg::Pose>(
    gazebo::msgs::ConvertIgn(image.pose()));

  detected_parts_.clear();
  tray_contents_msg_.parts.clear();

  for (int i = 0; i < image.model_size(); i++) {
    const auto & lc_model = image.model(i);

    std::string model_name = lc_model.name();

    // name of kit tray model is "kit_tray_XX_YY" where XX indicates 
    // the marker_id for the tray
    if (model_name.find("kit_tray") != std::string::npos){
      std::string id_string = model_name.substr(9, 2);
      kit_tray_.id = std::stoi(id_string);
      kit_tray_.pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(gazebo::msgs::ConvertIgn(lc_model.pose()));
    }
    
    // Check to see if model is a part 
    for ( const auto &type : part_types_ ) {
      ariac_msgs::msg::PartPose part_pose;
      
      if (model_name.find(type.first) != std::string::npos) {
        part_pose.part.type = type.second;

        // Determine part color
        for ( const auto &color : part_colors_ ) {
          if (model_name.find(color.first) != std::string::npos) {
            part_pose.part.color = color.second;

            // Get pose 
            part_pose.pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(gazebo::msgs::ConvertIgn(lc_model.pose()));

            detected_parts_.insert({model_name, part_pose});
            tray_contents_msg_.parts.push_back(part_pose.part);
          }
        }
      }
    }
  }

  tray_contents_pub_->publish(tray_contents_msg_);
}

void AGVTraySensorPluginPrivate::PerformQualityCheck(
  ariac_msgs::srv::PerformQualityCheck::Request::SharedPtr request,
  ariac_msgs::srv::PerformQualityCheck::Response::SharedPtr response) 
{
  // Check if order id is valid
  response->valid_id = false;

  ariac_msgs::msg::Order req_order;
  for (const ariac_msgs::msg::Order order: orders_){
    if (request->task_id == order.id){
      response->valid_id = true;
      req_order = order;
      break;
    }    
  }
  if (!response->valid_id) {
    RCLCPP_INFO(ros_node_->get_logger(), "ID is not valid");
    return;
  }

  // Check to see if the correct agv
  if (req_order.kitting_task.agv_number != std::stoi(sensor_num_)) {
    RCLCPP_INFO(ros_node_->get_logger(), "Wrong AGV for this order");
    response->valid_id = false;
    return;
  }

  // Check to see that the correct tray is on the AGV
  if (req_order.kitting_task.tray_id == kit_tray_.id) {
    response->incorrect_tray = false;
  } else {
    RCLCPP_INFO(ros_node_->get_logger(), "Tray is not correct for this order");
    response->incorrect_tray = true;
    return;
  }

  // Determine where each part on tray is located
  TrayParts parts_on_tray = LocatePartsOnTray();

  response->all_passed = true;
  // Check each quadrant with a part for issue
  for (ariac_msgs::msg::KittingPart part : req_order.kitting_task.parts) {    
    ariac_msgs::msg::QualityIssue issue;

    //  see if a part is present in that quadrant
    std::map<int, ariac_msgs::msg::PartPose>::iterator it = parts_on_tray.parts.find(part.quadrant);
    if (it == parts_on_tray.parts.end()){
      issue.missing_part = true;
      response->all_passed = false; 
    }

    // Check to see if the part type is correct
    if (it->second.part.type != part.part.type) {
      issue.incorrect_part_type = true;
      response->all_passed = false;    
    }

    // Check to see if the part color is correct
    if (it->second.part.color != part.part.color) {
      issue.incorrect_part_color = true;
      response->all_passed = false;  
    }

    //  check to see if the part is flipped
    if (CheckFlipped(kit_tray_, it->second)) {
      issue.flipped_part = true;
      response->all_passed = false;  
    }

    // TODO check order to see if the part should be reported faulty
    if (faulty_part_info_[request->task_id][part.quadrant]) {
      // Check if first check of faulty part
      if (faulty_part_names_[request->task_id][part.quadrant] == "")
      {
        faulty_part_names_[request->task_id][part.quadrant] = parts_on_tray.part_names[part.quadrant];
        issue.faulty_part = true;
        response->all_passed = false;      
      } else if (faulty_part_names_[request->task_id][part.quadrant] == parts_on_tray.part_names[part.quadrant]) {
        // Same part is still there
        issue.faulty_part = true;
        response->all_passed = false;
      }
    }

    if (part.quadrant == ariac_msgs::msg::KittingPart::QUADRANT1)
      response->quadrant1 = issue;
    else if (part.quadrant == ariac_msgs::msg::KittingPart::QUADRANT2)
      response->quadrant2 = issue;
    else if (part.quadrant == ariac_msgs::msg::KittingPart::QUADRANT3)
      response->quadrant3 = issue;
    else if (part.quadrant == ariac_msgs::msg::KittingPart::QUADRANT4)
      response->quadrant4 = issue;
  }

}

void AGVTraySensorPluginPrivate::FillOrders()
{
  // Create fake order for testing
  ariac_msgs::msg::Order test_order;
  test_order.id = "Kjri813s";
  test_order.type = ariac_msgs::msg::Order::KITTING;
  test_order.priority = false;

  ariac_msgs::msg::KittingTask task;
  task.tray_id = 3;
  task.agv_number = 4;
  task.destination = ariac_msgs::msg::KittingTask::WAREHOUSE;

  ariac_msgs::msg::KittingPart part1;
  part1.part.type = ariac_msgs::msg::Part::PUMP;
  part1.part.color = ariac_msgs::msg::Part::GREEN;
  part1.quadrant = ariac_msgs::msg::KittingPart::QUADRANT1;

  ariac_msgs::msg::KittingPart part2;
  part2.part.type = ariac_msgs::msg::Part::SENSOR;
  part2.part.color = ariac_msgs::msg::Part::GREEN;
  part2.quadrant = ariac_msgs::msg::KittingPart::QUADRANT3;

  task.parts.push_back(part1);
  task.parts.push_back(part2);

  test_order.kitting_task = task;

  orders_.push_back(test_order);

  // Enter faulty part info
  std::map<int, bool> faulty_info;
  faulty_info.insert({{ariac_msgs::msg::KittingPart::QUADRANT3}, true});
  faulty_info.insert({{ariac_msgs::msg::KittingPart::QUADRANT1}, false});

  std::map<int, std::string> faulty_names;
  faulty_names.insert({{ariac_msgs::msg::KittingPart::QUADRANT3}, ""});
  faulty_names.insert({{ariac_msgs::msg::KittingPart::QUADRANT1}, ""});

  faulty_part_info_.insert({test_order.id, faulty_info});
  faulty_part_names_.insert({test_order.id, faulty_names});

}

TrayParts AGVTraySensorPluginPrivate::LocatePartsOnTray()
{ 
  TrayParts parts_on_tray;

  // Convert tray pose to KDL frame
  KDL::Frame sensor_to_tray;
  tf2::fromMsg(kit_tray_.pose, sensor_to_tray);

  for (const auto tray_part: detected_parts_) {
    // Determine transform between tray and part
    KDL::Frame sensor_to_part;
    tf2::fromMsg(tray_part.second.pose, sensor_to_part);

    KDL::Frame tray_to_part;
    tray_to_part = sensor_to_tray.Inverse() * sensor_to_part;

    geometry_msgs::msg::Pose part_pose_on_tray = tf2::toMsg(tray_to_part);
  
    // Determine part's quadrant
    if (part_pose_on_tray.position.x < 0 && part_pose_on_tray.position.y > 0) {
      parts_on_tray.parts.insert({ariac_msgs::msg::KittingPart::QUADRANT1, tray_part.second});
      parts_on_tray.part_names.insert({ariac_msgs::msg::KittingPart::QUADRANT1, tray_part.first});
    
    } else if (part_pose_on_tray.position.x > 0 && part_pose_on_tray.position.y > 0) {
      parts_on_tray.parts.insert({ariac_msgs::msg::KittingPart::QUADRANT2, tray_part.second});
      parts_on_tray.part_names.insert({ariac_msgs::msg::KittingPart::QUADRANT2, tray_part.first});
    
    } else if (part_pose_on_tray.position.x < 0 && part_pose_on_tray.position.y < 0) {
      parts_on_tray.parts.insert({ariac_msgs::msg::KittingPart::QUADRANT3, tray_part.second});
      parts_on_tray.part_names.insert({ariac_msgs::msg::KittingPart::QUADRANT3, tray_part.first});
    
    } else {
      parts_on_tray.parts.insert({ariac_msgs::msg::KittingPart::QUADRANT4, tray_part.second});
      parts_on_tray.part_names.insert({ariac_msgs::msg::KittingPart::QUADRANT4, tray_part.first});
    }
  }

  return parts_on_tray;
}

bool AGVTraySensorPluginPrivate::CheckFlipped(
  ariac_msgs::msg::KitTrayPose tray_pose, 
  ariac_msgs::msg::PartPose part_pose) 
{
  // Convert both to kdl frames
  KDL::Frame world_to_sensor;
  tf2::fromMsg(sensor_pose_, world_to_sensor);

  KDL::Frame sensor_to_tray;
  tf2::fromMsg(tray_pose.pose, sensor_to_tray);

  KDL::Frame sensor_to_part;
  tf2::fromMsg(part_pose.pose, sensor_to_part);

  KDL::Frame world_to_tray = world_to_sensor * sensor_to_tray;
  KDL::Frame world_to_part = world_to_sensor * sensor_to_part;

  KDL::Vector up(0, 0, 1);

  // Create vector that represents the +z direction of the part and tray
  KDL::Vector tray_z = world_to_tray * up;
  KDL::Vector part_z = world_to_part * up;

  // Calculate the angle between the two vectors
  double angle = KDL::acos(KDL::dot(tray_z, part_z)/(tray_z.Norm()*part_z.Norm()));

  // Return that the part is flipped if angle is greater than ~10deg
  if (angle > -0.2 && angle < 0.2){
    return false;
  } else {
    return true;
  }
}


GZ_REGISTER_SENSOR_PLUGIN(AGVTraySensorPlugin)

}  // namespace ariac_plugins
