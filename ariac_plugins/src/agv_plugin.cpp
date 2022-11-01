#include <gazebo/physics/Model.hh>
#include <ariac_plugins/agv_plugin.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace ariac_plugins
{
/// Class to hold private data members (PIMPL pattern)
class AGVPluginPrivate
{
public:
  /// Connection to world update event. Callback is called while this is alive.
  gazebo::event::ConnectionPtr update_connection_;

  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;

};

AGVPlugin::AGVPlugin()
: impl_(std::make_unique<AGVPluginPrivate>())
{
}

AGVPlugin::~AGVPlugin()
{
}

void AGVPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Create ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

  // Create a connection so the OnUpdate function is called at every simulation iteration. 
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&AGVPlugin::OnUpdate, this));
}

void AGVPlugin::OnUpdate()
{
    
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(AGVPlugin)
}  // namespace ariac_plugins
