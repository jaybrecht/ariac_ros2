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


#include <ariac_sensors/logical_camera.hpp>
#include <gazebo/sensors/LogicalCameraSensor.hh>
#include <ariac_msgs/msg/logical_camera_image.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>

#include <memory>

namespace ariac_sensors
{

class LogicalCameraPrivate
{
public:
  /// Node for ros communication
  gazebo_ros::Node::SharedPtr ros_node_;
  /// Publish for logical camera message
  rclcpp::Publisher<ariac_msgs::msg::LogicalCameraImage>::SharedPtr pub_;
  /// LogicalCameraImage message modified each update
  ariac_msgs::msg::LogicalCameraImage::SharedPtr msg_;
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

LogicalCamera::LogicalCamera()
: impl_(std::make_unique<LogicalCameraPrivate>())
{
}

LogicalCamera::~LogicalCamera()
{
}

void LogicalCamera::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  this->impl_->sensor_ = std::dynamic_pointer_cast<gazebo::sensors::LogicalCameraSensor>(_sensor);
  this->impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  std::string topic_name;

  if (!_sdf->HasElement("camera_name")) {
    topic_name = "logical_camera";
  } else {
    topic_name = _sdf->Get<std::string>("camera_name");
  }

  impl_->pub_ =
    impl_->ros_node_->create_publisher<ariac_msgs::msg::LogicalCameraImage>(
    topic_name, rclcpp::SensorDataQoS());

  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Created [%s] topic", impl_->pub_->get_topic_name());

  // Set list of models to publish
  this->impl_->models_to_publish_ = {"pump", "battery", "regulator", "sensor"};

  this->impl_->colors_ = {"red", "green", "blue"};

  // Create message to be reused
  impl_->msg_ = std::make_shared<ariac_msgs::msg::LogicalCameraImage>();

  impl_->sensor_update_event_ = impl_->sensor_->ConnectUpdated(
    std::bind(&LogicalCameraPrivate::OnUpdate, impl_.get()));
}

void LogicalCameraPrivate::OnUpdate()
{
  const auto & image = this->sensor_->Image();

  msg_->pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(
    gazebo::msgs::ConvertIgn(image.pose()));

  msg_->models.clear();

  for (int i = 0; i < image.model_size(); i++) {
    ariac_msgs::msg::Model model;

    const auto & lc_model = image.model(i);

    for(std::string type : models_to_publish_){
      std::string name = lc_model.name();
      if (name.find(type) != std::string::npos) {
        model.type = type;
        
        for(std::string color : colors_){
          if (name.find(color) != std::string::npos) {
            model.color = color;
          }
        }

        model.pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(gazebo::msgs::ConvertIgn(lc_model.pose()));
        msg_->models.push_back(model);
        break;
      }

    }
  }

  this->pub_->publish(*msg_);
}

GZ_REGISTER_SENSOR_PLUGIN(LogicalCamera)

}  // namespace ariac_sensors
