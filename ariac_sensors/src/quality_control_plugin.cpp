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

#include <ariac_msgs/srv/perform_quality_check.hpp>

#include <memory>

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

  /// Publish latest logical camera data to ROS
  void OnUpdate();
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

  // Set list of models to publish
  this->impl_->models_to_publish_ = {"pump", "battery", "regulator", "sensor"};

  this->impl_->colors_ = {"red", "green", "blue"};

  impl_->sensor_update_event_ = impl_->sensor_->ConnectUpdated(
    std::bind(&QualityControlPluginPrivate::OnUpdate, impl_.get()));
}

void QualityControlPluginPrivate::OnUpdate()
{
  const auto & image = this->sensor_->Image();

  // camera_pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(
  //   gazebo::msgs::ConvertIgn(image.pose()));

  // msg_->models.clear();

  // for (int i = 0; i < image.model_size(); i++) {
  //   ariac_msgs::msg::Model model;

  //   const auto & lc_model = image.model(i);

  //   for(std::string type : models_to_publish_){
  //     std::string name = lc_model.name();
  //     if (name.find(type) != std::string::npos) {
  //       model.type = type;
        
  //       for(std::string color : colors_){
  //         if (name.find(color) != std::string::npos) {
  //           model.color = color;
  //         }
  //       }

  //       model.pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(gazebo::msgs::ConvertIgn(lc_model.pose()));
  //       msg_->models.push_back(model);
  //       break;
  //     }

  //   }
  // }

  // this->pub_->publish(*msg_);
}

GZ_REGISTER_SENSOR_PLUGIN(QualityControlPlugin)

}  // namespace ariac_plugins
