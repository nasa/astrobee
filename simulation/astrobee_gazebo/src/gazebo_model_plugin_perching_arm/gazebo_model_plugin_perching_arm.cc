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

// Joint state message
#include <sensor_msgs/JointState.h>

// Gazebo includes
#include <astrobee_gazebo/astrobee_gazebo.h>

// STL includes
#include <string>

namespace gazebo {

/* The perching arm has two DYNAMIXEL XM 430 intelligent motors. They
   run an internal PID control loop to achieve velocity or position 
   control. The PID maps: a delta position -> target PWM (max: 885). */
class GazeboModelPluginPerchingArm : public FreeFlyerModelPlugin {
 public:
  // Constructor
  GazeboModelPluginPerchingArm() : FreeFlyerModelPlugin("joint_state", false), rate_(10.5) {}

  // Destructor
  ~GazeboModelPluginPerchingArm() {}

 protected:
  // Called when the plugin is loaded into the simulator
  void LoadCallback(ros::NodeHandle *nh, physics::ModelPtr model, sdf::ElementPtr sdf) {
    // If we specify a frame name different to our sensor tag name
    if (sdf->HasElement("rate"))
      rate_ = sdf->Get<double>("rate");
    // Get a list of joints
    joints_.push_back(GetModel()->GetJoint("arm_proximal_joint"));
    joints_.push_back(GetModel()->GetJoint("arm_proximal_joint"));
    joints_.push_back(GetModel()->GetJoint("arm_distal_joint"));
    joints_.push_back(GetModel()->GetJoint("gripper_left_proximal_joint"));
    joints_.push_back(GetModel()->GetJoint("gripper_left_distal_joint"));
    joints_.push_back(GetModel()->GetJoint("gripper_right_proximal_joint"));
    joints_.push_back(GetModel()->GetJoint("gripper_right_distal_joint"));
    // Setup message
    msg_.header.frame_id =  GetModel()->GetName();
    msg_.name.resize(joints_.size());
    msg_.position.resize(joints_.size());
    msg_.velocity.resize(joints_.size());
    msg_.effort.resize(joints_.size());
    // Create a join state publisher
    pub_ = nh->advertise < sensor_msgs::JointState > ("joint_states", 1, true);
    // Called before each iteration of simulated world update
    next_tick_ = GetWorld()->GetSimTime();
    update_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboModelPluginPerchingArm::UpdateCallback, this));
  }

  // Called on simulation reset
  void Reset() {
    next_tick_ = GetWorld()->GetSimTime();
  }

  // Called on every discrete time tick in the simulated world
  void UpdateCallback() {
    // Throttle callback rate
    if (GetWorld()->GetSimTime() < next_tick_)
      return;
    next_tick_ += 1.0 / rate_;
    // Publish the message
    msg_.header.stamp = ros::Time::now();
    for (size_t i = 0; i < joints_.size(); i++) {
      msg_.name[i] = joints_[i]->GetName();
      msg_.position[i] = joints_[i]->GetAngle(0).Radian();
      msg_.velocity[i] = joints_[i]->GetVelocity(0);
      msg_.effort[i] = joints_[i]->GetForce(0);
    }
    pub_.publish(msg_);
  }

 private:
  double rate_;
  common::Time next_tick_;
  ros::Publisher pub_;
  event::ConnectionPtr update_;
  physics::Joint_V joints_;
  sensor_msgs::JointState msg_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboModelPluginPerchingArm)

}   // namespace gazebo
