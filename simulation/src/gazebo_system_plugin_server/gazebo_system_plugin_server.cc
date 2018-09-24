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

#include <ros/ros.h>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <functional>

namespace gazebo {

class SystemPluginServer : public SystemPlugin {
 public:
  ~SystemPluginServer() {}

  void Load(int /*_argc*/, char ** /*_argv*/) {}

  void Init() {
    gzmsg << "SystemPluginServer loaded..." << std::endl;
    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM("ROS has not been initialized");
      return;
    }
    subscriber_ = event::Events::ConnectWorldUpdateBegin(std::bind(
      &SystemPluginServer::OnWorldUpdate, this, std::placeholders::_1));
  }

  void OnWorldUpdate(const common::UpdateInfo & info) {
    ros::NodeHandle nh;
    double simulation_speed = 1.0;
    if (!nh.getParam("/simulation_speed", simulation_speed)) {
      gzwarn << "Sim speed not specified. Trying real-time." << std::endl;
      return;
    }
    simulation_speed *= 62.5;
    gzmsg << "Setting target update rate to " << simulation_speed << std::endl;
    physics::WorldPtr world = physics::get_world();
    // Gazebo 7.x -> 9.x migration
    // physics::PhysicsEnginePtr engine = world->GetPhysicsEngine();
    physics::PhysicsEnginePtr engine = world->Physics();
    // end Gazebo 7.x -> 9.x migration
    engine->SetRealTimeUpdateRate(simulation_speed);
    // Gazebo 7.x -> 9.x migration
    // event::Events::DisconnectWorldUpdateBegin(subscriber_);
    subscriber_.reset();
    // end Gazebo 7.x -> 9.x migration
  }

 private:
  event::ConnectionPtr subscriber_;
};

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(SystemPluginServer)

}   // namespace gazebo
