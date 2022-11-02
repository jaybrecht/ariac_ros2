#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <ariac_plugins/agv_plugin.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <ariac_msgs/srv/move_agv.hpp>

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

  gazebo::physics::ModelPtr model_;
  gazebo::physics::JointPtr agv_joint_;

  std::string agv_number_;
  double kitting_location_ = 0.0;
  double front_assembly_station_ = 5.6;
  double back_assembly_station_ = 10.6;
  double max_velocity_ = 2.0;
  double goal_position_;

  // Velocity publisher
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;
  std_msgs::msg::Float64MultiArray velocity_msg_;

  // Move service
  rclcpp::Service<ariac_msgs::srv::MoveAGV>::SharedPtr move_service_;

  void PublishVelocity(double vel);
  void MoveAGV( 
    ariac_msgs::srv::MoveAGV::Request::SharedPtr req,
    ariac_msgs::srv::MoveAGV::Response::SharedPtr res);
  void MoveToGoal(double goal);

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

  impl_->model_ = model;

  impl_->agv_number_ = sdf->GetElement("agv_number")->Get<std::string>();

  impl_->agv_joint_ = model->GetJoint(impl_->agv_number_ + "_joint");
  if (!impl_->agv_joint_) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "AGV joint not found, unable to start conveyor plugin");
    return;
  }

  // Register velocity publisher
  std::string vel_topic = "/" + impl_->agv_number_ + "/velocity_controller/commands";
  impl_->velocity_pub_ = impl_->ros_node_->create_publisher<std_msgs::msg::Float64MultiArray>(vel_topic, 10);

  // Register move service
  impl_->move_service_ = impl_->ros_node_->create_service<ariac_msgs::srv::MoveAGV>(
      "/ariac/move_"+impl_->agv_number_, 
      std::bind(
      &AGVPluginPrivate::MoveAGV, impl_.get(),
      std::placeholders::_1, std::placeholders::_2));

  // Create a connection so the OnUpdate function is called at every simulation iteration. 
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&AGVPlugin::OnUpdate, this));
}

void AGVPlugin::OnUpdate()
{
  // impl_->PublishVelocity();
}


void AGVPluginPrivate::PublishVelocity(double vel)
{
  // RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Publishing velocity: " << vel);
  velocity_msg_.data.clear();
  velocity_msg_.data.push_back(vel);
  velocity_pub_->publish(velocity_msg_);
}


void AGVPluginPrivate::MoveAGV(
  ariac_msgs::srv::MoveAGV::Request::SharedPtr req,
  ariac_msgs::srv::MoveAGV::Response::SharedPtr res)
{
  res->success = false;

  if (req->location == ariac_msgs::srv::MoveAGV::Request::KITTING){
    RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Moving " << agv_number_ << " to kitting station");

    this->MoveToGoal(kitting_location_);    

    res->success = true;
    return;
  }
  else if (req->location == ariac_msgs::srv::MoveAGV::Request::ASSEMBLY_FRONT){
    RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Moving " << agv_number_ << " to front assembly station");

    this->MoveToGoal(front_assembly_station_);   

    res->success = true;
    return;
  }
  else if (req->location == ariac_msgs::srv::MoveAGV::Request::ASSEMBLY_BACK){
    RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Moving " << agv_number_ << " to back assembly station");

    this->MoveToGoal(back_assembly_station_);   

    res->success = true;
    return;
  }
  else {
    RCLCPP_WARN(ros_node_->get_logger(), "Location not recognized");
  }
  
}

void AGVPluginPrivate::MoveToGoal(double goal){
  int direction = 1;
  if (agv_joint_->Position(0) > goal)
  {
    direction = -1;
  }
    
  if (direction > 0){
    this->PublishVelocity(max_velocity_);

    while(agv_joint_->Position(0) < goal){}

    this->PublishVelocity(0.0);
  }
  else {
    this->PublishVelocity(-max_velocity_);

    while(agv_joint_->Position(0) > goal){}

    this->PublishVelocity(0.0);
  }
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(AGVPlugin)
}  // namespace ariac_plugins
