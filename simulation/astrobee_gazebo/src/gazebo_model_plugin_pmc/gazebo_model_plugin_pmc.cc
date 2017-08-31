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

// FreeFlyer messages
#include <ff_hw_msgs/PmcCommand.h>

// Autocode inclide
#include <gnc_autocode/blowers.h>

// Gazebo includes
#include <astrobee_gazebo/astrobee_gazebo.h>

// STL includes
#include <string>

namespace gazebo {

// This class is a plugin that calls the GNC autocode to predict
// the forced to be applied to the rigid body
class GazeboModelPluginPmc : public FreeFlyerModelPlugin {
 public:
  GazeboModelPluginPmc() : FreeFlyerModelPlugin(NODE_PMC_ACTUATOR), rate_(62.5) {}

  virtual ~GazeboModelPluginPmc() {}

 protected:
  // Called when the plugin is loaded into the simulator
  void LoadCallback(ros::NodeHandle *nh, physics::ModelPtr model, sdf::ElementPtr sdf) {
    // If we specify a frame name different to our sensor tag name
    if (sdf->HasElement("rate"))
      rate_ = sdf->Get<double>("rate");

    // Create a null command
    ff_hw_msgs::PmcState null_state;
    null_state.motor_speed = 0;
    null_state.nozzle_positions = {25, 25, 25, 25, 25, 25};
    null_command_.header.frame_id = GetFrame();
    null_command_.states.push_back(null_state);
    null_command_.states.push_back(null_state);

    // Now register to be called back every time FAM has new wrench
    sub_ = nh->subscribe(TOPIC_HARDWARE_PMC_COMMAND, 1,
      &GazeboModelPluginPmc::CommandCallback, this);

    // Create a watchdog timer to ensure the PMC commands are set
    timer_ = nh->createTimer(ros::Duration(20.0/62.5),
      &GazeboModelPluginPmc::WatchdogCallback, this, false, true);

    // Called before each iteration of simulated world update
    next_tick_ = GetWorld()->GetSimTime();
    update_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboModelPluginPmc::WorldUpdateCallback, this));
  }

  // Called on simulation reset
  virtual void Reset() {
    next_tick_ = GetWorld()->GetSimTime();
  }

  // This is called whenever the controller has new force/torque to apply
  void CommandCallback(ff_hw_msgs::PmcCommand const& msg) {
    // Immediately reset the watchdog timer
    timer_.stop();
    // ros::getGlobalCallbackQueue()->clear();
    timer_.start();
    // Send the command
    SendCommand(msg);
  }

  // If this is *ever* called, it means a FAM command was not received - note that we
  // are using a single-threaded spinner so at most one callback is processed at any time
  // which means that we wont have race conditions on commands_
  void WatchdogCallback(ros::TimerEvent const& event) {
    // Immediately reset the watchdog timer
    timer_.stop();
    // ros::getGlobalCallbackQueue()->clear();
    timer_.start();
    // Update the null command time
    null_command_.header.stamp = ros::Time::now();
    // set the blower speed to zero in case we do not receive messages from FAM
    SendCommand(null_command_);
  }

  // Send a command to the PMCs to those specifi
  void SendCommand(ff_hw_msgs::PmcCommand const& msg) {
    if (msg.states.size() != 2)
      return;
    // Set the impeller and nozzle values to those given in the message
    for (size_t i = 0; i < msg.states.size(); i++) {
      blowers_.states_[i].impeller_cmd = msg.states[i].motor_speed;
      for (size_t j = 0; j < 6; j++)
        blowers_.states_[i].servo_cmd[j]
          = static_cast < float > (msg.states[i].nozzle_positions[j]);
    }
  }

  // Called on each sensor update event
  void WorldUpdateCallback() {
    // Throttle callback rate
    if (GetWorld()->GetSimTime() >= next_tick_) {
      next_tick_ += 1.0 / rate_;
      // Set the angular velocity
      blowers_.SetAngularVelocity(GetLink()->GetRelativeAngularVel().x,
        GetLink()->GetRelativeAngularVel().y, GetLink()->GetRelativeAngularVel().z);
      // Set the battery voltage
      blowers_.SetBatteryVoltage(14.0);
      // Step the system
      blowers_.Step();
      // Extract and apply the force and torque for the blowers
      force_.x = blowers_.states_[0].force_B[0] + blowers_.states_[1].force_B[0];
      force_.y = blowers_.states_[0].force_B[1] + blowers_.states_[1].force_B[1];
      force_.z = blowers_.states_[0].force_B[2] + blowers_.states_[1].force_B[2];
      torque_.x = blowers_.states_[0].torque_B[0] + blowers_.states_[1].torque_B[0];
      torque_.y = blowers_.states_[0].torque_B[1] + blowers_.states_[1].torque_B[1];
      torque_.z = blowers_.states_[0].torque_B[2] + blowers_.states_[1].torque_B[2];
    }
    // Apply the force and torque to the model
    GetLink()->AddRelativeForce(force_);
    GetLink()->AddRelativeTorque(torque_);
  }

 private:
  double rate_;
  common::Time next_tick_;
  ros::Subscriber sub_;
  ros::Timer timer_;
  math::Vector3 force_;
  math::Vector3 torque_;
  gnc_autocode::GncBlowersAutocode blowers_;
  ff_hw_msgs::PmcCommand null_command_;
  event::ConnectionPtr update_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboModelPluginPmc)

}   // namespace gazebo
