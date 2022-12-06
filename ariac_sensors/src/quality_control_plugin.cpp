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
#include <ariac_msgs/msg/part.hpp>
#include <ariac_msgs/msg/part_pose.hpp>
#include <ariac_msgs/msg/kit_tray_pose.hpp>

#include <memory>
#include <map>

namespace ariac_sensors
{

class QualityControlPluginPrivate
{
public:
  /// Node for ros communication
  gazebo_ros::Node::SharedPtr ros_node_;
  /// LogicalCamera sensor this plugin is attached to
  gazebo::sensors::LogicalCameraSensorPtr sensor_;
  /// Event triggered when sensor updates
  gazebo::event::ConnectionPtr sensor_update_event_;
  /// List of models that the logical camera will publish
  std::vector<std::string> models_to_publish_;
  std::vector<std::string> colors_;

  std::map<std::string, int> part_types_;
  std::map<std::string, int> part_colors_;

  /// Lists of parts and trays
  std::vector<ariac_msgs::msg::PartPose> tray_parts_;
  std::vector<ariac_msgs::msg::KitTrayPose> kit_trays_;

  geometry_msgs::msg::Pose sensor_pose_;

  rclcpp::Service<ariac_msgs::srv::PerformQualityCheck>::SharedPtr quality_check_service_;

  /// Publish latest logical camera data to ROS
  void OnUpdate();

  // 
  void PerformQualityCheck(
    ariac_msgs::srv::PerformQualityCheck::Request::SharedPtr,
    ariac_msgs::srv::PerformQualityCheck::Response::SharedPtr
  );
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

  std::string sensor_num;
  if (!_sdf->HasElement("sensor_num")) {
    sensor_num = "0";
  } else {
    sensor_num = _sdf->Get<std::string>("sensor_num");
  }

  this->impl_->part_types_ = {{"battery", 10}, {"pump", 11}, {"sensor", 12}, {"regulator", 13}}; 
  this->impl_->part_colors_ = {{"red", 0}, {"green", 1}, {"blue", 2}, {"orange", 3}, {"purple", 4}};

  // Create quality control service 
  impl_->quality_check_service_ = this->impl_->ros_node_->create_service<ariac_msgs::srv::PerformQualityCheck>(
  "/ariac/perform_quality_check_agv" + sensor_num, 
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
  kit_trays_.clear();

  for (int i = 0; i < image.model_size(); i++) {
    const auto & lc_model = image.model(i);

    std::string model_name = lc_model.name();

    // name of kit tray model is "kit_tray_XX_YY" where XX indicates 
    // the marker_id for the tray
    if (model_name.find("kit_tray") != std::string::npos){
      ariac_msgs::msg::KitTrayPose tray;
      std::string id_string = model_name.substr(9, 2);
      tray.id = std::stoi(id_string);
      tray.pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(gazebo::msgs::ConvertIgn(lc_model.pose()));

      kit_trays_.push_back(tray);
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
  ariac_msgs::srv::PerformQualityCheck::Request::SharedPtr,
  ariac_msgs::srv::PerformQualityCheck::Response::SharedPtr) 
{
  RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Num Parts:" << tray_parts_.size());
}

GZ_REGISTER_SENSOR_PLUGIN(QualityControlPlugin)

}  // namespace ariac_plugins
