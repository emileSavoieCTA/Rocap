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

#include <math.h>
#include <memory>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/Visual.hh>


#include <gazebo/physics/Model.hh>
#include <gazebo_rope.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ignition/math.hh>
#include <ignition/math/Pose3.hh>


namespace gazebo_plugins
{
/// Class to hold private data members (PIMPL pattern)
class GazeboRopePrivate
{
public:
  /// Connection to world update event. Callback is called while this is alive.
  gazebo::event::ConnectionPtr update_connection_;

  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;

  std::string armName;
  std::string pulleyName;
  std::string baselinkName;
  std::string visualName;
  std::string color;

  double pullStrength;

  gazebo::rendering::VisualPtr visualPtr;
  gazebo::rendering::DynamicLines* line;
  gazebo::transport::NodePtr node;
  gazebo::transport::PublisherPtr visPub;
  gazebo::physics::ModelPtr modelPtr;
  gazebo::physics::LinkPtr armLinkPtr;
  gazebo::physics::LinkPtr pulleyLinkPtr;
  gazebo::physics::LinkPtr baseLinkPtr;
};

GazeboRope::GazeboRope()
: impl_(std::make_unique<GazeboRopePrivate>())
{
}

GazeboRope::~GazeboRope()
{
}

void GazeboRope::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr sdf)
{
  // // Create a GazeboRos node instead of a common ROS node.
  // // Pass it SDF parameters so common options like namespace and remapping
  // // can be handled.
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);
  impl_->modelPtr = _parent;
  bool initError = false;

  this->impl_->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->impl_->node->Init(impl_->modelPtr->GetWorld()->Name());
  this->impl_->visPub = this->impl_->node->Advertise<gazebo::msgs::Visual>("~/visual", 10);

  std::string baselinkName;
  if (!sdf->HasElement("baselink")) {
    RCLCPP_ERROR_STREAM(impl_->ros_node_->get_logger(), "rope plugin missing <baselink>\n\n\n\n\n");
    initError = true;
  } else {
    baselinkName = sdf->GetElement("baselink")->Get<std::string>();
    RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "geting baselink name :" <<  baselinkName);
    this->impl_->baseLinkPtr = _parent->GetLink(baselinkName);
  }

  if (!this->impl_->baseLinkPtr) {
    RCLCPP_ERROR_STREAM(impl_->ros_node_->get_logger(), "gazebo_rope_viz plugin error: bodyName: "
                                                    << baselinkName << " does not exist");
    initError = true;
  }

  if (!sdf->HasElement("arm")) {
    RCLCPP_ERROR_STREAM(impl_->ros_node_->get_logger(), "rope plugin missing <arm>\n\n\n\n\n");
    initError = true;
  } else {
    this->impl_->armName = sdf->GetElement("arm")->Get<std::string>();
    this->impl_->armLinkPtr = _parent->GetLink(this->impl_->armName);
  }

  if (!this->impl_->armLinkPtr) {
    RCLCPP_ERROR_STREAM(impl_->ros_node_->get_logger(), "gazebo_rope_viz plugin error: bodyName: "
                                                  << this->impl_->armName << " does not exist");
    initError = true;
  }

  if (!sdf->HasElement("pulley")) {
    RCLCPP_ERROR_STREAM(impl_->ros_node_->get_logger(), "rope plugin missing <pulley>\n");
    initError = true;
  } else {
    this->impl_->pulleyName = sdf->GetElement("pulley")->Get<std::string>();
    this->impl_->pulleyLinkPtr = _parent->GetLink(this->impl_->pulleyName);
  }

  if (!this->impl_->pulleyLinkPtr) {
    RCLCPP_ERROR_STREAM(impl_->ros_node_->get_logger(), "gazebo_rope_viz plugin error: bodyName: "
                                                  << this->impl_->armName << " does not exist");
    initError = true;
  }

  if (!sdf->HasElement("name")) {
    this->impl_->visualName = "cylinder";
  } else {
    this->impl_->visualName = sdf->GetElement("name")->Get<std::string>();
  }

  if (!sdf->HasElement("color")) {
    this->impl_->color = "Gazebo/RedGlow";
  } else {
    this->impl_->color = sdf->GetElement("color")->Get<std::string>();
  }

  if (!sdf->HasElement("pull_strength")) {
    this->impl_->pullStrength = 0;
  } else {
    this->impl_->pullStrength = sdf->GetElement("pull_strength")->Get<double>();
  }

  this->impl_->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->impl_->node->Init(impl_->modelPtr->GetWorld()->Name());
  this->impl_->visPub = this->impl_->node->Advertise<gazebo::msgs::Visual>("~/visual", 10);

  if (!initError) {
    // Create a connection so the OnUpdate function is called at every simulation
    // iteration.
    impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboRope::OnUpdate, this));
  } else {
    RCLCPP_ERROR_STREAM(impl_->ros_node_->get_logger(),
                    "some error occure pluging plugin with stop");
  }
}

void GazeboRope::OnUpdate()
{
  auto framePose = this->impl_->armLinkPtr->WorldPose();

  auto refFrame = this->impl_->baseLinkPtr->WorldPose();

  auto targetPose = this->impl_->pulleyLinkPtr->WorldPose();

  // compute dir vector
  ignition::math::Vector3d dirVec = targetPose.Pos()-framePose.Pos();
  double dis = dirVec.Length();
  dirVec = dirVec.Normalize();

  // compute visual center
  ignition::math::Vector3d offset = dirVec*dis/2;
  ignition::math::Vector3d visualPos = framePose.Pos()+offset;

  // compute angle
  double roll = 0;
  double pitch = getPitch(dirVec) + M_PI/2;  // +PI/2 to  align rope with dir vector
  double yaw = getYaw(dirVec);

  // apply force on link
  this->impl_->armLinkPtr->AddForce(impl_->pullStrength*dirVec);
  this->impl_->pulleyLinkPtr->AddForce(impl_->pullStrength*-dirVec);

  ignition::math::Pose3d poseWorldFrame(visualPos.X(),
                                        visualPos.Y(),
                                        visualPos.Z(),
                                        roll,
                                        -pitch,
                                        yaw);

  ignition::math::Pose3d poseRelativeToModel = poseWorldFrame-refFrame;

  updateRopeVisual(poseRelativeToModel, 0.01, dis);
}

double GazeboRope::getYaw(ignition::math::Vector3d dirVec)
{
  if (dirVec.Y() != 0.0 || dirVec.X() != 0.0) {
    return atan2 (dirVec.Y(), dirVec.X());
  }

  return 0.0;
}

double GazeboRope::getPitch(ignition::math::Vector3d dirVec)
{
  ignition::math::Vector3d xyVec(dirVec.X(), dirVec.Y(), 0);
  if (dirVec.Z() != 0.0 || xyVec.Length() != 0.0) {
    return atan2 (dirVec.Z(), xyVec.Length());
  }

  return 0.0;
}

void GazeboRope::updateRopeVisual(ignition::math::Pose3d pose, double radius, double length)
{
  gazebo::msgs::Visual visualMsg;

  // Set the visual's name. This should be unique.
  visualMsg.set_name(this->impl_->visualName);

  // Set the visual's parent. This visual will be attached to the parent

  visualMsg.set_parent_name(impl_->modelPtr->GetScopedName());

  // Create a cylinder
  gazebo::msgs::Geometry *geomMsg = visualMsg.mutable_geometry();
  geomMsg->set_type(gazebo::msgs::Geometry::CYLINDER);
  geomMsg->mutable_cylinder()->set_radius(radius);
  geomMsg->mutable_cylinder()->set_length(length);

  // Set the material to be bright red
  visualMsg.mutable_material()->mutable_script()->set_name(
    impl_->color);

  gazebo::msgs::Set(visualMsg.mutable_pose(), pose);

  // Don't cast shadows
  visualMsg.set_cast_shadows(false);

  this->impl_->visPub->Publish(visualMsg);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRope)
}  // namespace gazebo_plugins
