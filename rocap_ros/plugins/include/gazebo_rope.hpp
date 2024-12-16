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

#ifndef GAZEBO_PLUGINS_GAZEBO_ROPE_HPP_
#define GAZEBO_PLUGINS_GAZEBO_ROPE_HPP_

#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"

#include <gazebo/common/Plugin.hh>

#include <ignition/math/Pose3.hh>

// For std::unique_ptr, could be removed
#include <memory>

namespace gazebo_plugins
{
// Forward declaration of private data class.
class GazeboRopePrivate;

/// Example ROS-powered Gazebo plugin with some useful boilerplate.
/// \details This is a `ModelPlugin`, but it could be any supported Gazebo plugin type, such as
/// System, Visual, GUI, World, Sensor, etc.
class GazeboRope : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboRope();

  /// Destructor
  virtual ~GazeboRope();

  /// Gazebo calls this when the plugin is loaded.
  /// \param[in] model Pointer to parent model. Other plugin types will expose different entities,
  /// such as `gazebo::sensors::SensorPtr`, `gazebo::physics::WorldPtr`,
  /// `gazebo::rendering::VisualPtr`, etc.
  /// \param[in] sdf SDF element containing user-defined parameters.
  void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr sdf) override;
  void updateRopeVisual(ignition::math::Pose3d pose, double radius, double length);
  double getYaw(ignition::math::Vector3d dirVec);
  double getPitch(ignition::math::Vector3d dirVec);

protected:
  /// Optional callback to be called at every simulation iteration.
  virtual void OnUpdate();

private:
  /// Recommended PIMPL pattern. This variable should hold all private
  /// data members.
  std::unique_ptr<GazeboRopePrivate> impl_;
};
}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_PLUGINS_GAZEBO_ROPE_HPP_
