/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

// Gazebo includes
#include <astrobee_gazebo/astrobee_gazebo.h>

namespace gazebo {

// This class is a plugin that calls the GNC autocode to predict
// the forced to be applied to the rigid body
class GazeboModelPluginDrag : public FreeFlyerModelPlugin {
 public:
  GazeboModelPluginDrag() : FreeFlyerModelPlugin("gazebo_drag", ""),
    coefficient_(1.05), area_(0.092903), density_(1.225) {}

  ~GazeboModelPluginDrag() {
    #if GAZEBO_MAJOR_VERSION > 7
    update_.reset();
    #else
    event::Events::DisconnectWorldUpdateBegin(update_);
    #endif
  }

 protected:
  // Called when the plugin is loaded into the simulator
  void LoadCallback(NodeHandle &nh, physics::ModelPtr model, sdf::ElementPtr sdf) {
    // Drag coefficient
    if (sdf->HasElement("coefficient"))
      coefficient_ = sdf->Get<double>("coefficient");
    // Cross-sectional area
    if (sdf->HasElement("area"))
      area_ = sdf->Get<double>("area");
    // Air density
    if (sdf->HasElement("density"))
      density_ = sdf->Get<double>("density");
    // Called before each iteration of simulated world update
    #if GAZEBO_MAJOR_VERSION > 7
    next_tick_ = GetWorld()->SimTime();
    #else
    next_tick_ = GetWorld()->GetSimTime();
    #endif
    update_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboModelPluginDrag::WorldUpdateCallback, this));
  }

  // Called on simulation reset
  void Reset() {
    #if GAZEBO_MAJOR_VERSION > 7
    next_tick_ = GetWorld()->SimTime();
    #else
    next_tick_ = GetWorld()->GetSimTime();
    #endif
  }

  // Called on each sensor update event
  void WorldUpdateCallback() {
    // Calculate drag
    #if GAZEBO_MAJOR_VERSION > 7
    drag_ = GetLink()->RelativeLinearVel();
    vel_ = drag_.Length();
    #else
    drag_ = GetLink()->GetRelativeLinearVel();
    vel_ = drag_.GetLength();
    #endif
    drag_ = -0.5 * coefficient_ * area_ * density_
           * vel_ * vel_ * drag_.Normalize();

    // Apply the force and torque to the model
    GetLink()->AddRelativeForce(drag_);
  }

 private:
  double coefficient_, area_, density_, vel_;              // Drag parameters
  common::Time next_tick_;
#if GAZEBO_MAJOR_VERSION > 7
  ignition::math::Vector3d drag_;
#else
  math::Vector3 drag_;
#endif
  event::ConnectionPtr update_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboModelPluginDrag)

}   // namespace gazebo
