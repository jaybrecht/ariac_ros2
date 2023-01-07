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

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <ariac_plugins/human_teleport_plugin.hpp> //LB
#include <ariac_msgs/srv/teleport_human.hpp>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
//LB these two includes are test
#include <ignition/math/Vector3.hh>
#include <ignition/math.hh>

#include <memory>

namespace ariac_plugins
{
/// Class to hold private data members (PIMPL pattern)
class HumanTeleportPluginPrivate
{
public:
  /// Connection to world update event. Callback is called while this is alive.
  gazebo::event::ConnectionPtr update_connection_;

  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// The joint that controls the movement of the human robot.
  gazebo::physics::JointPtr human_joint_;

// LB: exemplo de variaveis do AGVPlugin
  gazebo::physics::ModelPtr model_;  
  gazebo::physics::WorldPtr world_;

  // Teleport service
  rclcpp::Service<ariac_msgs::srv::TeleportHuman>::SharedPtr teleport_service_;

  void TeleportHuman(
    ariac_msgs::srv::TeleportHuman::Request::SharedPtr req,
    ariac_msgs::srv::TeleportHuman::Response::SharedPtr res);

};

HumanTeleportPlugin::HumanTeleportPlugin()
: impl_(std::make_unique<HumanTeleportPluginPrivate>())
{
}

HumanTeleportPlugin::~HumanTeleportPlugin()
{
}

//void HumanTeleportPlugin::Load(gazebo::rendering::VisualPtr visual, sdf::ElementPtr sdf)
//public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
void HumanTeleportPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Create a GazeboRos node instead of a common ROS node.
  // Pass it SDF parameters so common options like namespace and remapping
  // can be handled.
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);
//  std::string name = sdf->GetElement("human")->Get<std::string>();

//  impl_->visual_ = visual;
  impl_->model_ = model;

  // impl_->agv_number_ = sdf->GetElement("agv_number")->Get<std::string>();

  // impl_->agv_joint_ = model->GetJoint(impl_->agv_number_ + "_joint");
  // if (!impl_->agv_joint_) {
  //   RCLCPP_ERROR(impl_->ros_node_->get_logger(), "AGV joint not found, unable to start conveyor plugin");
  //   return;
  // }

  // Create belt joint / LB: created but not used
  // impl_->human_joint_ = model->GetJoint("base_footprint"); //human_joint");

  // if (!impl_->human_joint_) {
  //   RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Human joint not found, unable to start Teleport plugin");
  //   return;
  // }

  impl_->teleport_service_ = impl_->ros_node_->create_service<ariac_msgs::srv::TeleportHuman>(
    "/ariac/teleport_human", 
//    "/ariac/" + name + "_teleport_human", 
    std::bind(
      &HumanTeleportPluginPrivate::TeleportHuman, impl_.get(),
      std::placeholders::_1, std::placeholders::_2));

  // Create a connection so the OnUpdate function is called at every simulation iteration. 
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&HumanTeleportPlugin::OnUpdate, this));
//   gazebo::msgs::Visual vis;
//   gazebo::msgs::Geometry geometry;
//   geometry.
//   vis.geometry()

//   visual->Mesh

  // The model pointer gives you direct access to the physics object,
  // for example:
  //RCLCPP_INFO(impl_->ros_node_->get_logger(), impl_->visual_->Name().c_str());

  // Create a connection so the OnUpdate function is called at every simulation
  // iteration. Remove this call, the connection and the callback if not needed.
  // impl_->update_connection_ = gazebo::event::Events::ConnectPreRender(
  //   std::bind(&HumanTeleportPlugin::OnUpdate, this));
}

void HumanTeleportPlugin::OnUpdate()
{
  // Publish status at rate
  //rclcpp::Time now = impl_->ros_node_->get_clock()->now();
  // if (now - impl_->last_publish_time_ >= rclcpp::Duration(0, impl_->update_ns_)) {
  //   impl_->PublishStatus();
  //   impl_->last_publish_time_ = now; 
  // }
}

//LB: this is the function to be changed
void HumanTeleportPluginPrivate::TeleportHuman(
  ariac_msgs::srv::TeleportHuman::Request::SharedPtr req,
  ariac_msgs::srv::TeleportHuman::Response::SharedPtr res) 
{
  res->success = false;

  //https://dev.to/jemaloqiu/ignition-gazebo-math-library-30fc

  // Bellow is sample from: https://answers.gazebosim.org//question/26408/setworldpose-of-several-models-very-slow-updates/
  
  RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Teleporting human to orign");

  //double time = world_->SimTime().Double();
  //ignition::math::Pose3d pose(0.0, 0.0, sin(time), 0.0, 0.0, 0.0);  // = orig_pose;
  ignition::math::Pose3d pose(-15.3, -10.0, 0.0, 0.0, 0.0, 0.0);  

  model_->SetWorldPose(pose);

  //LB: other possibilities
  //model_->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
  //this->human_joint_->SetPosition(0, 0);

  res->success = true;
  return;
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(HumanTeleportPlugin)
}  // namespace ariac_plugins
