#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <ariac_plugins/conveyor_belt_plugin.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ariac_msgs/srv/conveyor_belt_control.hpp>

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
  double max_velocity_;

  /// Position limit of belt joint to reset 
  double limit_;

  /// Service for enabling the vacuum gripper
  rclcpp::Service<ariac_msgs::srv::ConveyorBeltControl>::SharedPtr enable_service_;

  /// Callback for enable service
  void SetConveyorPower(
    ariac_msgs::srv::ConveyorBeltControl::Request::SharedPtr,
    ariac_msgs::srv::ConveyorBeltControl::Response::SharedPtr);
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
  impl_->max_velocity_ = sdf->GetElement("max_velocity")->Get<double>();

  // Set limit (m)
  impl_->limit_ = impl_->belt_joint_->UpperLimit();

    // Register enable service
  impl_->enable_service_ = impl_->ros_node_->create_service<ariac_msgs::srv::ConveyorBeltControl>(
      "set_conveyor_power", 
      std::bind(
        &ConveyorBeltPluginPrivate::SetConveyorPower, impl_.get(),
        std::placeholders::_1, std::placeholders::_2));

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

void ConveyorBeltPluginPrivate::SetConveyorPower(
  ariac_msgs::srv::ConveyorBeltControl::Request::SharedPtr req,
  ariac_msgs::srv::ConveyorBeltControl::Response::SharedPtr res)
{
  res->success = false;
  if (req->power >= 0 && req->power <= 100) {
    belt_velocity_ = max_velocity_ * (req->power / 100);
    res->success = true;
  }
  else{
    RCLCPP_WARN(ros_node_->get_logger(), "Conveyor power must be between 0 and 100");
  }
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ConveyorBeltPlugin)
}  // namespace ariac_plugins
