// Copyright 2020 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <ariac_sensors/quality_control_plugin.hpp>
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

#include <tf2_kdl/tf2_kdl.h>
#include <tf2/convert.h>
#include <kdl/frames.hpp>

#include <memory>
#include <map>

namespace ariac_sensors
{

// KDL::Frame PoseToFrame(const geometry_msgs::msg::Pose& msg)
// {
//   KDL::Frame out;

//   out.p[0] = msg.position.x;
//   out.p[1] = msg.position.y;
//   out.p[2] = msg.position.z;

//   std::cout << "Converting quaternion to rotation matrix" << std::endl;
//   out.M = KDL::Rotation::Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);

//   return out;
// }


class QualityControlPluginPrivate
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
  std::vector<ariac_msgs::msg::PartPose> tray_parts_;
  ariac_msgs::msg::KitTrayPose kit_tray_;

  /// List of orders
  std::vector<ariac_msgs::msg::Order> orders_;

  geometry_msgs::msg::Pose sensor_pose_;

  rclcpp::Service<ariac_msgs::srv::PerformQualityCheck>::SharedPtr quality_check_service_;

  /// Publish latest logical camera data to ROS
  void OnUpdate();

  // 
  void PerformQualityCheck(
    ariac_msgs::srv::PerformQualityCheck::Request::SharedPtr,
    ariac_msgs::srv::PerformQualityCheck::Response::SharedPtr
  );

  void FillOrders();

  std::map<int, ariac_msgs::msg::PartPose> LocatePartsOnTray();
  
  bool CheckFlipped(
    ariac_msgs::msg::KitTrayPose tray_pose, 
    ariac_msgs::msg::PartPose part_pose);
};

QualityControlPlugin::QualityControlPlugin()
: impl_(std::make_unique<QualityControlPluginPrivate>())
{
}

QualityControlPlugin::~QualityControlPlugin()
{
}

void QualityControlPlugin::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
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
    &QualityControlPluginPrivate::PerformQualityCheck, this->impl_.get(),
    std::placeholders::_1, std::placeholders::_2));

  impl_->sensor_update_event_ = impl_->sensor_->ConnectUpdated(
    std::bind(&QualityControlPluginPrivate::OnUpdate, this->impl_.get()));
}

void QualityControlPluginPrivate::OnUpdate()
{
  const auto & image = sensor_->Image();

  sensor_pose_ = gazebo_ros::Convert<geometry_msgs::msg::Pose>(
    gazebo::msgs::ConvertIgn(image.pose()));

  tray_parts_.clear();

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

            tray_parts_.push_back(part_pose);
          }
        }
      }
    }
  }
}

void QualityControlPluginPrivate::PerformQualityCheck(
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
  std::map<int, ariac_msgs::msg::PartPose> parts_on_tray = LocatePartsOnTray();
  // std::map<int, ariac_msgs::msg::PartPose> parts_on_tray;

  response->all_passed = true;
  // Check each quadrant with a part for issue
  for (ariac_msgs::msg::KittingPart part : req_order.kitting_task.parts) {    
    ariac_msgs::msg::QualityIssue issue;

    //  see if a part is present in that quadrant
    std::map<int, ariac_msgs::msg::PartPose>::iterator it = parts_on_tray.find(part.quadrant);
    if (it == parts_on_tray.end()){
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

void QualityControlPluginPrivate::FillOrders()
{
  // Create fake order for testing
  ariac_msgs::msg::Order test_order;
  test_order.id = "Kjri813s";
  test_order.type = ariac_msgs::msg::Order::KITTING;
  test_order.priority = false;

  ariac_msgs::msg::KittingTask task;
  task.tray_id = 1;
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
}

std::map<int, ariac_msgs::msg::PartPose> QualityControlPluginPrivate::LocatePartsOnTray()
{  
  std::map<int, ariac_msgs::msg::PartPose> parts_on_tray;

  // Convert tray pose to KDL frame
  KDL::Frame sensor_to_tray;
  tf2::fromMsg(kit_tray_.pose, sensor_to_tray);

  for (const ariac_msgs::msg::PartPose part: tray_parts_) {
    // Determine transform between tray and part
    KDL::Frame sensor_to_part;
    tf2::fromMsg(part.pose, sensor_to_part);

    KDL::Frame tray_to_part;
    tray_to_part = sensor_to_tray.Inverse() * sensor_to_part;

    geometry_msgs::msg::Pose part_pose_on_tray = tf2::toMsg(tray_to_part);
  
    // Determine part's quadrant
    if (part_pose_on_tray.position.x < 0 && part_pose_on_tray.position.y > 0) {
      parts_on_tray.insert({ariac_msgs::msg::KittingPart::QUADRANT1, part});
    } else if (part_pose_on_tray.position.x > 0 && part_pose_on_tray.position.y > 0) {
      parts_on_tray.insert({ariac_msgs::msg::KittingPart::QUADRANT2, part});
    } else if (part_pose_on_tray.position.x < 0 && part_pose_on_tray.position.y < 0) {
      parts_on_tray.insert({ariac_msgs::msg::KittingPart::QUADRANT3, part});
    } else {
      parts_on_tray.insert({ariac_msgs::msg::KittingPart::QUADRANT4, part});
    }
  }

  

  return parts_on_tray;
}

bool QualityControlPluginPrivate::CheckFlipped(
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

  // RCLCPP_INFO_STREAM(ros_node_->get_logger(), "angle between vectors" << angle);
  // Return that the part is flipped if angle is greater than ~10deg
  if (angle > -0.2 && angle < 0.2){
    RCLCPP_INFO(ros_node_->get_logger(), "Not flipped");
    return false;
  } else {
    return true;
  }
}


GZ_REGISTER_SENSOR_PLUGIN(QualityControlPlugin)

}  // namespace ariac_plugins
