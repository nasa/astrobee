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

// ROS includes
#include <ros/ros.h>

// Gazebo includes
#include <astrobee_gazebo/astrobee_gazebo.h>

namespace gazebo {

// This class is a plugin that calls the GNC autocode to predict
// the forced to be applied to the rigid body
class GazeboModelPluginDrag : public FreeFlyerModelPlugin {
 public:
  GazeboModelPluginDrag() : FreeFlyerModelPlugin("gazebo_drag", ""),
    coefficient_(1.05), area_(0.092903), density_(1.225) {}

  ~GazeboModelPluginDrag() {}

 protected:
  // Called when the plugin is loaded into the simulator
  void LoadCallback(
    ros::NodeHandle *nh, physics::ModelPtr model, sdf::ElementPtr sdf) {
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
    next_tick_ = GetWorld()->GetSimTime();
    update_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboModelPluginDrag::WorldUpdateCallback, this));
  }

  // Called on simulation reset
  virtual void Reset() {
    next_tick_ = GetWorld()->GetSimTime();
  }

  // Called on each sensor update event
  void WorldUpdateCallback() {
    // Calculate drag
    drag_ = GetLink()->GetRelativeLinearVel();
    drag_ = -0.5 * coefficient_ * area_ * density_
          * drag_.GetLength() * drag_.GetLength() * drag_.Normalize();
    // Apply the force and torque to the model
    GetLink()->AddRelativeForce(drag_);
  }

 private:
  double coefficient_, area_, density_;              // Drag parameters
  common::Time next_tick_;
  math::Vector3 drag_;
  event::ConnectionPtr update_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboModelPluginDrag)

}   // namespace gazebo
