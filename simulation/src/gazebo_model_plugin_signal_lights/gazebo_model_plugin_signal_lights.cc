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

// FSW nodelet
#include <ff_common/ff_names.h>

// Services
#include <ff_msgs/srv/set_streaming_lights.hpp>

// STL includes
#include <string>

namespace gazebo {

class GazeboModelPluginSignalLights : public FreeFlyerModelPlugin {
 public:
  GazeboModelPluginSignalLights() : FreeFlyerModelPlugin(
    "signal_lights", "", true) {}

  ~GazeboModelPluginSignalLights() {}

 protected:
  // Called when the plugin is loaded into the simulator
  void LoadCallback(NodeHandle& nh,
    physics::ModelPtr model, sdf::ElementPtr sdf) {
    // this service is a special case for when we need to light
    // two AMBER leds on each side only when we are streaming
    // live video
    streaming_service_ = FF_CREATE_SERVICE(nh, ff_msgs::srv::SetStreamingLights, SERVICE_STREAMING_LIGHTS,
                            std::bind(&GazeboModelPluginSignalLights::StreamingLightsCallback, this,
                            std::placeholders::_1, std::placeholders::_2));
  }

  // Manage the extrinsics based on the sensor type
  bool ExtrinsicsCallback(geometry_msgs::TransformStamped const* tf) {
    return true;
  }

  bool StreamingLightsCallback(
      const std::shared_ptr<ff_msgs::srv::SetStreamingLights::Request> request,
      std::shared_ptr<ff_msgs::srv::SetStreamingLights::Response> response) {
    // Send successful response
    response->success = true;
    return true;
  }

  rclcpp::Service<ff_msgs::srv::SetStreamingLights>::SharedPtr streaming_service_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboModelPluginSignalLights)

}   // namespace gazebo
