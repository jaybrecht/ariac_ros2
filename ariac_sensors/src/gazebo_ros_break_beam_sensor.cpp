// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include <boost/make_shared.hpp>
#include <boost/variant.hpp>
#include <gazebo/transport/transport.hh>
#include <ariac_sensors/gazebo_ros_break_beam_sensor.hpp>
#include <gazebo_ros/conversions/sensor_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ariac_msgs/msg/break_beam_status.hpp>

#include <string>
#include <algorithm>
#include <limits>
#include <memory>

namespace ariac_sensors
{

class GazeboRosBreakBeamSensorPrivate
{
public:
  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;

  ariac_msgs::msg::BreakBeamStatus status_msg_;

  rclcpp::Publisher<ariac_msgs::msg::BreakBeamStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<ariac_msgs::msg::BreakBeamStatus>::SharedPtr change_pub_;

  /// TF frame output is published in
  std::string frame_name_;

  /// Subscribe to gazebo's laserscan, calling the appropriate callback based on output type
  void SubscribeGazeboLaserScan();

  /// Publish a sensor_msgs/LaserScan message from a gazebo laser scan
  void ReadLaserScan(ConstLaserScanStampedPtr & _msg);

  /// Gazebo transport topic to subscribe to for laser scan
  std::string sensor_topic_;

  /// Gazebo node used to subscribe to laser scan
  gazebo::transport::NodePtr gazebo_node_;

  /// Gazebo subscribe to parent sensor's laser scan
  gazebo::transport::SubscriberPtr laser_scan_sub_;
};

GazeboRosBreakBeamSensor::GazeboRosBreakBeamSensor()
: impl_(std::make_unique<GazeboRosBreakBeamSensorPrivate>())
{
}

GazeboRosBreakBeamSensor::~GazeboRosBreakBeamSensor()
{
  // Must release subscriber and then call fini on node to remove it from topic manager.
  impl_->laser_scan_sub_.reset();
  if (impl_->gazebo_node_) {
    impl_->gazebo_node_->Fini();
  }
  impl_->gazebo_node_.reset();
}

void GazeboRosBreakBeamSensor::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  // Create ros_node configured from sdf
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  // Get QoS profile for the publisher
  rclcpp::QoS pub_qos = qos.get_publisher_qos("~/out", rclcpp::SensorDataQoS().reliable());

  // Get tf frame for output
  impl_->frame_name_ = gazebo_ros::SensorFrameID(*_sensor, *_sdf);

  // Get topic names from xml
  std::string status_topic;
  std::string change_topic;

  if (!_sdf->HasElement("status_topic")) {
    status_topic = "break_beam_status";
  } else {
    status_topic = _sdf->Get<std::string>("status_topic");
  }

  if (!_sdf->HasElement("change_topic")) {
    change_topic = "break_beam_change";
  } else {
    change_topic = _sdf->Get<std::string>("change_topic");
  }

  // Create publishers
  impl_->status_pub_ = impl_->ros_node_->create_publisher<ariac_msgs::msg::BreakBeamStatus>(status_topic, pub_qos);
  impl_->change_pub_ = impl_->ros_node_->create_publisher<ariac_msgs::msg::BreakBeamStatus>(change_topic, pub_qos);

  // Create gazebo transport node and subscribe to sensor's laser scan
  impl_->gazebo_node_ = boost::make_shared<gazebo::transport::Node>();
  impl_->gazebo_node_->Init(_sensor->WorldName());

  impl_->sensor_topic_ = _sensor->Topic();
  impl_->SubscribeGazeboLaserScan();
}

void GazeboRosBreakBeamSensorPrivate::SubscribeGazeboLaserScan()
{
  laser_scan_sub_ = gazebo_node_->Subscribe(
      sensor_topic_, &GazeboRosBreakBeamSensorPrivate::ReadLaserScan, this);
}

void GazeboRosBreakBeamSensorPrivate::ReadLaserScan(ConstLaserScanStampedPtr & _msg)
{
  // Convert Laser scan to ROS LaserScan
  auto ls = gazebo_ros::Convert<sensor_msgs::msg::LaserScan>(*_msg, 0.0);
  // Set tf frame
  status_msg_.header.frame_id = frame_name_;
  status_msg_.header.stamp = ros_node_->get_clock()->now();

  bool object_detected = false;
  bool publish_change = false;

  for(float distance : ls.ranges) {
    if (distance > 0.0 && distance < 1.0) {
        // RCLCPP_INFO(ros_node_->get_logger(), "Object detected");
        object_detected = true;
        break;
    }
  }

  if (status_msg_.object_detected != object_detected) {
    publish_change = true;
  }

  status_msg_.object_detected = object_detected;

  // Publish output
  if (publish_change) {
    change_pub_->publish(status_msg_);
  }
  status_pub_->publish(status_msg_);
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosBreakBeamSensor)

}  // namespace ariac_sensors