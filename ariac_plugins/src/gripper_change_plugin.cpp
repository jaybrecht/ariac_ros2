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

#include <gazebo/rendering/Visual.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <ariac_plugins/gripper_change_plugin.hpp>
#include <ariac_msgs/srv/change_gripper_color.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace ariac_plugins
{
/// Class to hold private data members (PIMPL pattern)
class GripperChangePluginPrivate
{
public:
  /// Connection to world update event. Callback is called while this is alive.
  gazebo::event::ConnectionPtr update_connection_;

  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;

  gazebo::rendering::VisualPtr visual_;

  ignition::math::Color color_;

  rclcpp::Service<ariac_msgs::srv::ChangeGripperColor>::SharedPtr color_change_service_;

  void ChangeGripperColor(
    ariac_msgs::srv::ChangeGripperColor::Request::SharedPtr,
    ariac_msgs::srv::ChangeGripperColor::Response::SharedPtr);

};

GripperChangePlugin::GripperChangePlugin()
: impl_(std::make_unique<GripperChangePluginPrivate>())
{
}

GripperChangePlugin::~GripperChangePlugin()
{
}

void GripperChangePlugin::Load(gazebo::rendering::VisualPtr visual, sdf::ElementPtr sdf)
{
  // Create a GazeboRos node instead of a common ROS node.
  // Pass it SDF parameters so common options like namespace and remapping
  // can be handled.
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);
  std::string name = sdf->GetElement("robot_name")->Get<std::string>();

  impl_->visual_ = visual;

  impl_->color_change_service_ = impl_->ros_node_->create_service<ariac_msgs::srv::ChangeGripperColor>(
    "/ariac/" + name + "_change_gripper_color", 
    std::bind(
      &GripperChangePluginPrivate::ChangeGripperColor, impl_.get(),
      std::placeholders::_1, std::placeholders::_2));

//   gazebo::msgs::Visual vis;
//   gazebo::msgs::Geometry geometry;
//   geometry.
//   vis.geometry()

//   visual->Mesh

  // The model pointer gives you direct access to the physics object,
  // for example:
  RCLCPP_INFO(impl_->ros_node_->get_logger(), impl_->visual_->Name().c_str());

  // Create a connection so the OnUpdate function is called at every simulation
  // iteration. Remove this call, the connection and the callback if not needed.
  // impl_->update_connection_ = gazebo::event::Events::ConnectPreRender(
  //   std::bind(&GripperChangePlugin::OnUpdate, this));
}

void GripperChangePlugin::OnUpdate()
{
  // Do something every simulation iteration
}

void GripperChangePluginPrivate::ChangeGripperColor(
  ariac_msgs::srv::ChangeGripperColor::Request::SharedPtr req,
  ariac_msgs::srv::ChangeGripperColor::Response::SharedPtr res) 
{
  res->success = false;

  // Check that all the values are between 0 and 1
  std::vector<bool> checks = {
    0 <= req->r <=1, 
    0 <= req->g <=1,
    0 <= req->b <=1,
    0 <= req->alpha <=1};

  if (std::find(checks.begin(), checks.end(), false) == checks.end()) 
  {
    color_.R(req->r);
    color_.G(req->g);
    color_.B(req->b);
    color_.A(req->alpha);

    visual_->SetDiffuse(color_);
    visual_->SetAmbient(color_);
    visual_->SetTransparency(1 - req->alpha);

    res->success = true;

  }
}

// Register this plugin with the simulator
GZ_REGISTER_VISUAL_PLUGIN(GripperChangePlugin)
}  // namespace ariac_plugins
