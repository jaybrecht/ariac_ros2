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

#include <gazebo_msgs/srv/delete_entity.hpp>

#include <ariac_plugins/object_disposal_plugin.hpp>

#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <map>
#include <memory>

namespace ariac_plugins
{
/// Class to hold private data members (PIMPL pattern)
class ObjectDisposalPluginPrivate
{
public:
  /// Connection to world update event. Callback is called while this is alive.
  gazebo::event::ConnectionPtr update_connection_;

  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;

  gazebo::transport::SubscriberPtr contact_sub_;
  gazebo::transport::NodePtr gznode_;

  std::vector<std::string> part_types_;

  gazebo::physics::ModelPtr model_;

  /// Service to delete model
  rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_model_client_;
};

ObjectDisposalPlugin::ObjectDisposalPlugin()
: impl_(std::make_unique<ObjectDisposalPluginPrivate>())
{
}

ObjectDisposalPlugin::~ObjectDisposalPlugin()
{
}

void ObjectDisposalPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Create a GazeboRos node instead of a common ROS node.
  // Pass it SDF parameters so common options like namespace and remapping
  // can be handled.
  impl_->model_ = model;
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

  impl_->part_types_ = {"battery", "regulator", "pump", "sensor"};

  gazebo::physics::WorldPtr world = impl_->model_->GetWorld();

  // Initialize a gazebo node and subscribe to the contacts for the vacuum gripper
  impl_->gznode_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  impl_->gznode_->Init(impl_->model_->GetWorld()->Name());

  // Get contact topic
  // std::string model_name = model->GetName();
  // std::string link_name = sdf->GetElement("link_name")->Get<std::string>();
  // std::string topic = "/gazebo/world/"+ model_name + "/" + link_name + "/bumper/contacts";
  std::string topic = "/gazebo/world/workcell_floor/floor/bumper/contacts";
  RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), topic);

  // Register client
  impl_->delete_model_client_ = impl_->ros_node_->create_client<gazebo_msgs::srv::DeleteEntity>(
      "/delete_entity");

  impl_->contact_sub_ = impl_->gznode_->Subscribe(topic, &ObjectDisposalPlugin::OnContact, this);
}

void ObjectDisposalPlugin::OnContact(ConstContactsPtr& _msg){
  std::string model_in_contact;

  bool removed = false;
  for (int i = 0; i < _msg->contact_size(); ++i) {
    // Find out which contact is the agv
    if (_msg->contact(i).collision1().find(impl_->model_->GetName()) != std::string::npos) {
      model_in_contact = _msg->contact(i).collision2();
    }
    else if (_msg->contact(i).collision2().find(impl_->model_->GetName()) != std::string::npos){
      model_in_contact = _msg->contact(i).collision1();
    }
    else {
      continue;
    }

    // Check if model is a part
    for (std::string &type : impl_->part_types_){
      if (model_in_contact.find(type) != std::string::npos){
        auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
        request->name = model_in_contact.substr(0, model_in_contact.find("::"));
        // RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Removing: " << request->name);
        impl_->delete_model_client_->async_send_request(request);
        removed = true;
        break;
      }
    }

    if (removed){
      break;
    }
  }
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ObjectDisposalPlugin)
}  // namespace ariac_plugins
