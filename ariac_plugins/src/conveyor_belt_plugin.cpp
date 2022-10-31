#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <ariac_plugins/conveyor_belt_plugin.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace ariac_plugins
{
/// Class to hold private data members (PIMPL pattern)
class ConveyorBeltPluginPrivate
{
public:
  /// Connection to world update event. Callback is called while this is alive.
  gazebo::event::ConnectionPtr update_connection_;

  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// The joint that controls the movement of the belt.
  gazebo::physics::JointPtr belt_joint_;

  /// Velocity of the belt joint
  double belt_velocity_;

  /// Position limit of belt joint to reset 
  double limit_;
};

ConveyorBeltPlugin::ConveyorBeltPlugin()
: impl_(std::make_unique<ConveyorBeltPluginPrivate>())
{
}

ConveyorBeltPlugin::~ConveyorBeltPlugin()
{
}

void ConveyorBeltPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Create ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

  // Create belt joint
  impl_->belt_joint_ = model->GetJoint("belt_joint");

  if (!impl_->belt_joint_) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Belt joint not found, unable to start conveyor plugin");
    return;
  }

  // Set velocity (m/s)
  impl_->belt_velocity_ = 0.2;

  // Set limit (m)
  impl_->limit_ = 0.4;

  // Create a connection so the OnUpdate function is called at every simulation iteration. 
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&ConveyorBeltPlugin::OnUpdate, this));
}

void ConveyorBeltPlugin::OnUpdate()
{
  impl_->belt_joint_->SetVelocity(0, impl_->belt_velocity_);

  double belt_position = impl_->belt_joint_->Position(0);

  if (belt_position >= impl_->limit_){
    impl_->belt_joint_->SetPosition(0, 0);
  }
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ConveyorBeltPlugin)
}  // namespace ariac_plugins
