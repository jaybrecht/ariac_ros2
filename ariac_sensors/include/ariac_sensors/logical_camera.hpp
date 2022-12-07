// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef LOGICAL_CAMERA_HPP_
#define LOGICAL_CAMERA_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace ariac_sensors
{

class LogicalCameraPrivate;

/// Plugin to attach to a gazebo LogicalCamera sensor and publish ROS message of output
class LogicalCamera : public gazebo::SensorPlugin
{
public:
  /// Constructor.
  LogicalCamera();
  /// Destructor.
  virtual ~LogicalCamera();

  // Documentation Inherited
  void Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;

private:
  /// Private data pointer
  std::unique_ptr<LogicalCameraPrivate> impl_;
};

}  // namespace ariac_sensors

#endif  // LOGICAL_CAMERA_HPP_
