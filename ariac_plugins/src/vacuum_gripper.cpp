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
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/transport/Subscriber.hh>
#include <gazebo/transport/Node.hh>
#include <ariac_plugins/vacuum_gripper.hpp>
#include <ariac_msgs/srv/vacuum_gripper_control.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <map>
#include <memory>

namespace ariac_plugins
{
/// Class to hold private data members (PIMPL pattern)
class VacuumGripperPrivate
{
public:
  /// Connection to world update event. Callback is called while this is alive.
  gazebo::event::ConnectionPtr update_connection_;

  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;

  bool enabled_;
  gazebo::physics::LinkPtr gripper_link;

  gazebo::transport::SubscriberPtr contact_sub_;
  gazebo::transport::NodePtr gznode_;

  /// Service for enabling the vacuum gripper
  rclcpp::Service<ariac_msgs::srv::VacuumGripperControl>::SharedPtr enable_service_;

  gazebo::physics::ModelPtr model_;
  std::map<std::string, gazebo::physics::CollisionPtr> collisions_;

  std::vector<std::string> pickable_part_types;

  bool CheckModelContact(ConstContactsPtr&);

  /// Callback for enable service
  void EnableGripper(
    ariac_msgs::srv::VacuumGripperControl::Request::SharedPtr,
    ariac_msgs::srv::VacuumGripperControl::Response::SharedPtr);

};

VacuumGripper::VacuumGripper()
: impl_(std::make_unique<VacuumGripperPrivate>())
{
}

VacuumGripper::~VacuumGripper()
{
}

void VacuumGripper::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Create a GazeboRos node instead of a common ROS node.
  // Pass it SDF parameters so common options like namespace and remapping
  // can be handled.
  impl_->model_ = model;
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

  // Initialize a gazebo node and subscribe to the contacts for the vacuum gripper
  impl_->gznode_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  impl_->gznode_->Init(impl_->model_->GetWorld()->Name());

  // Get gripper link
  std::string link_name = sdf->GetElement("gripper_link")->Get<std::string>();
  impl_->gripper_link = impl_->model_->GetLink(link_name);
  
  std::string topic = "/gazebo/world/floor_robot/floor_gripper/bumper/contacts";
  impl_->contact_sub_ = impl_->gznode_->Subscribe(topic, &VacuumGripper::OnContact, this);

  impl_->pickable_part_types = {"battery", "regulator", "pump", "sensor"};

  // Register enable service
  impl_->enable_service_ = impl_->ros_node_->create_service<ariac_msgs::srv::VacuumGripperControl>(
      "enable_gripper", 
      std::bind(
      &VacuumGripperPrivate::EnableGripper, impl_.get(),
      std::placeholders::_1, std::placeholders::_2));

  // The model pointer gives you direct access to the physics object,
  // for example:
  // RCLCPP_INFO(impl_->ros_node_->get_logger(), model->GetName().c_str());

  // Create a connection so the OnUpdate function is called at every simulation
  // iteration. Remove this call, the connection and the callback if not needed.
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&VacuumGripper::OnUpdate, this));
}

void VacuumGripper::OnUpdate()
{
  // if (enabled_){
  //   // List collision

  // }
  // Do something every simulation iteration
}

void VacuumGripper::OnContact(ConstContactsPtr& _msg){
  if (impl_->enabled_) {
    if (impl_->CheckModelContact(_msg)){
      // Create link 
    }
  }
}

bool VacuumGripperPrivate::CheckModelContact(ConstContactsPtr& msg){
  std::string part_in_contact;
  std::string part_type;
  int min_contacts = 4;

  for (int i = 0; i < msg->contact_size(); ++i) {
    // Find out which contact is the robot
    if (msg->contact(i).collision1().find("robot") != std::string::npos) {
      part_in_contact = msg->contact(i).collision2();
    }
    else if (msg->contact(i).collision2().find("robot") != std::string::npos){
      part_in_contact = msg->contact(i).collision1();
    }
    else {
      continue;
    }

    // Check if part is in the pickable_list
    for (auto &type : pickable_part_types){
      if (part_in_contact.find(type) != std::string::npos){
        part_type = type;
        break;
      }
    }
    // Ignore contact otherwise
    if (part_type.empty()){
      continue;
    }
    
    // Check number of contacts
    if (!msg->contact(i).position_size() > min_contacts){
      continue;
    }

    //Check normals
    bool in_contact = false;
    for (int j = 0; j < msg->contact(i).normal_size(); ++j){
      ignition::math::Vector3d contact_normal = -1 * gazebo::msgs::ConvertIgn(msg->contact(i).normal(j));
      ignition::math::Vector3d gripper_normal =  gripper_link->WorldPose().Rot().RotateVector(ignition::math::Vector3d(0, 0, 1));

      double alignment = gripper_normal.Dot(contact_normal);

      if (alignment > 0.95) {
        in_contact = true;
      }
    }

    return in_contact;
    
  }

  return false;

}

void VacuumGripperPrivate::EnableGripper(
  ariac_msgs::srv::VacuumGripperControl::Request::SharedPtr req,
  ariac_msgs::srv::VacuumGripperControl::Response::SharedPtr res)
{
  res->success = false;
  if (req->enable) {
    if (!enabled_) {
      enabled_ = true;
      res->success = true;
    } else {
      RCLCPP_WARN(ros_node_->get_logger(), "Gripper is already enabled");
    }
  } else {
    if (enabled_) {
      enabled_ = false;
      res->success = true;
    } else {
      RCLCPP_WARN(ros_node_->get_logger(), "Gripper is already off");
    }
  }
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(VacuumGripper)
}  // namespace ariac_plugins
