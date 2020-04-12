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

namespace gazebo {

class WorldPluginSpeed : public WorldPlugin {
 public:
  // Constructor
  WorldPluginSpeed() {}

  // Destructor
  ~WorldPluginSpeed() {}

  void Load(physics::WorldPtr world, sdf::ElementPtr sdf) {
    world_ = world;
    // Check ROS is initialized
    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM("ROS has not been initialized");
      return;
    }
    nh_ = ros::NodeHandle("/");
    // Query the simulation speed
    double simulation_speed = 1.0;
    if (!nh_.getParam("/simulation_speed", simulation_speed)) {
      gzwarn << "Sim speed not specified. Trying real-time." << std::endl;
      return;
    }
    simulation_speed *= 125;
    // Set the simulation speed
    gzmsg << "Setting target update rate to " << simulation_speed << std::endl;
    physics::PhysicsEnginePtr engine = world->GetPhysicsEngine();
    engine->SetRealTimeUpdateRate(simulation_speed);
  }

 private:
  ros::NodeHandle nh_;
  ros::WallTimer timer_;
  physics::WorldPtr world_;
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(WorldPluginSpeed)

}   // namespace gazebo
