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

// STL includes
#include <string>

namespace gazebo {

class GazeboModelPluginLlpDiskMonitor : public FreeFlyerModelPlugin {
 public:
  GazeboModelPluginLlpDiskMonitor() : FreeFlyerModelPlugin(
    "llp_disk_monitor", "llp_disk_monitor", true) {}

  ~GazeboModelPluginLlpDiskMonitor() {}

 protected:
  // Called when the plugin is loaded into the simulator
  void LoadCallback(ros::NodeHandle *nh,
    physics::ModelPtr model, sdf::ElementPtr sdf) {}

  // Manage the extrinsics based on the sensor type
  bool ExtrinsicsCallback(geometry_msgs::TransformStamped const* tf) {
    return true;
  }
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboModelPluginLlpDiskMonitor)

}   // namespace gazebo
