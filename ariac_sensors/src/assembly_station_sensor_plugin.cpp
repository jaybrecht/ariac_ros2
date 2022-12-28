#include <ariac_sensors/assembly_station_sensor_plugin.hpp>

#include <gazebo/sensors/LogicalCameraSensor.hh>

#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>

#include <ariac_msgs/msg/part_pose.hpp>

#include <memory>

namespace ariac_sensors
{

class AssemblyStationSensorPluginPrivate
{
public:
  /// Node for ros communication
  gazebo_ros::Node::SharedPtr ros_node_;
  
  /// AssemblyStationSensorPlugin sensor this plugin is attached to
  gazebo::sensors::LogicalCameraSensorPtr sensor_;
  
  /// Event triggered when sensor updates
  gazebo::event::ConnectionPtr sensor_update_event_;
  
  /// List of models that the logical camera will publish
  std::vector<std::string> parts_to_publish_;
  std::vector<std::string> colors_;

  std::map<std::string, int> part_types_;
  std::map<std::string, int> part_colors_;

  /// Publish latest logical camera data to ROS
  void OnUpdate();
};

AssemblyStationSensorPlugin::AssemblyStationSensorPlugin()
: impl_(std::make_unique<AssemblyStationSensorPluginPrivate>())
{
}

AssemblyStationSensorPlugin::~AssemblyStationSensorPlugin()
{
}

void AssemblyStationSensorPlugin::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  impl_->sensor_ = std::dynamic_pointer_cast<gazebo::sensors::LogicalCameraSensor>(_sensor);
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  impl_->part_types_ = {
    {"battery", ariac_msgs::msg::Part::BATTERY},
    {"pump", ariac_msgs::msg::Part::PUMP},
    {"regulator", ariac_msgs::msg::Part::REGULATOR},
    {"sensor", ariac_msgs::msg::Part::SENSOR},
  };

  impl_->part_colors_ = {
    {"red", ariac_msgs::msg::Part::RED},
    {"green", ariac_msgs::msg::Part::GREEN},
    {"blue", ariac_msgs::msg::Part::BLUE},
    {"purple", ariac_msgs::msg::Part::PURPLE},
    {"orange", ariac_msgs::msg::Part::ORANGE},
  };

  impl_->sensor_update_event_ = impl_->sensor_->ConnectUpdated(
    std::bind(&AssemblyStationSensorPluginPrivate::OnUpdate, impl_.get()));
}

void AssemblyStationSensorPluginPrivate::OnUpdate()
{
  const auto & image = this->sensor_->Image();

  geometry_msgs::msg::Pose sensor_pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(
    gazebo::msgs::ConvertIgn(image.pose()));

  std::vector<ariac_msgs::msg::PartPose> parts;

  for (int i = 0; i < image.model_size(); i++) {

    const auto & lc_model = image.model(i);
    std::string name = lc_model.name();

    for(const auto &type : part_types_) {
      if (name.find(type.first) != std::string::npos) {
        ariac_msgs::msg::PartPose part;

        part.part.type = type.second;
        
        for(const auto &color : part_colors_) {
          if (name.find(color.first) != std::string::npos) {
            part.part.color = color.second;
          }
        }

        part.pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(gazebo::msgs::ConvertIgn(lc_model.pose()));

        parts.push_back(part);

        break;
      }
    }
  }
}

GZ_REGISTER_SENSOR_PLUGIN(AssemblyStationSensorPlugin)

}  // namespace ariac_sensors
