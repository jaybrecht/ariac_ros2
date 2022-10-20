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

#ifndef BREAK_BEAM_SENSOR_HPP_
#define BREAK_BEAM_SENSOR_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace ariac_sensors
{

class BreakBeamSensorPrivate;


class BreakBeamSensor : public gazebo::SensorPlugin
{
public:
  /// \brief Constructor
  BreakBeamSensor();

  /// \brief Destructor
  virtual ~BreakBeamSensor();

  // Documentation Inherited
  void Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

private:
  std::unique_ptr<BreakBeamSensorPrivate> impl_;
};

}  // namespace ariac_sensors

#endif  // BREAK_BEAM_SENSOR_HPP_