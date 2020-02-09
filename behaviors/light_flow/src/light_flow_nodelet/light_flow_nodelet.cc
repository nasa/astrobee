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

#include "../../include/light_flow.h"

#include <jsoncpp/json/allocator.h>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/reader.h>
#include <jsoncpp/json/value.h>

// Standard ROS includes
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

// FSW shared libraries
#include <config_reader/config_reader.h>

// FSW nodelet
#include <ff_util/ff_names.h>
#include <ff_util/ff_nodelet.h>

// Services
#include <ff_hw_msgs/ConfigureLEDGroup.h>
#include <ff_msgs/SetStreamingLights.h>
#include <ff_msgs/SignalState.h>

#include <cerrno>
#include <cstring>

namespace light_flow {

class LightFlowNodelet : public ff_util::FreeFlyerNodelet {
 public:
  LightFlowNodelet() : ff_util::FreeFlyerNodelet("light_flow") {}

  virtual ~LightFlowNodelet() {}

 protected:
  virtual void Initialize(ros::NodeHandle* nh) {
    // this service is a special case for when we need to light
    // two AMBER leds on each side only when we are streaming
    // live video
    streaming_service_ =
        nh->advertiseService(SERVICE_STREAMING_LIGHTS,
                             &LightFlowNodelet::StreamingLightsCallback, this);
    currentStreamingLightsState = false;

    pub_ = nh->advertise<ff_hw_msgs::ConfigureLEDGroup>(
        TOPIC_HARDWARE_SIGNAL_LIGHTS_CONFIG, 5);

    config_params.AddFile("behaviors/light_flow.config");
    if (!config_params.ReadFiles()) {
      ROS_ERROR("Could not open config file");
      return;
    }

    std::string folder;
    if (!config_params.GetStr("folder", &folder)) {
      ROS_ERROR("Could not get state file");
      return;
    }

    config_reader::ConfigReader::Table states(&config_params, "states");
    for (int i = 0; i < states.GetSize(); i++) {
      config_reader::ConfigReader::Table state;
      if (!states.GetTable(i + 1, &state)) {
        ROS_ERROR("Could not get state table");
        return;
      }

      std::string id;
      if (!state.GetStr("id", &id)) {
        ROS_ERROR("Could not get state id");
        return;
      }
      std::string file;
      if (!state.GetStr("file", &file)) {
        ROS_ERROR("Could not get state file");
        return;
      }

      std::ifstream t(folder + file);
      std::stringstream document;
      document << t.rdbuf();

      Json::Reader reader;
      Json::Value compiledLightFlow;
      if (!reader.parse(document.str(), compiledLightFlow)) {
        ROS_ERROR("Could not parse a lightflow json string");
        continue;
      } else {
        compileLightFlow(compiledLightFlow);
        ROS_INFO("Compiled a light flow proc successfully");
      }

      light_flow_states.emplace(id, compiledLightFlow);
    }

    sub_ = nh->subscribe(TOPIC_SIGNALS, 5, &LightFlowNodelet::ConfigureCallback,
                         this);
  }

  void ConfigureCallback(const ff_msgs::SignalState::ConstPtr& msg) {
    switch (msg->state) {
      case ff_msgs::SignalState::VIDEO_ON:
        RunLightFlow("VIDEO_ON");
        return;
      case ff_msgs::SignalState::VIDEO_OFF:
        RunLightFlow("VIDEO_OFF");
        return;
      case ff_msgs::SignalState::SUCCESS:
        RunLightFlow("SUCCESS");
        return;
      case ff_msgs::SignalState::ENTER_HATCHWAY:
        RunLightFlow("ENTER_HATCHWAY");
        return;
      case ff_msgs::SignalState::UNDOCK:
        RunLightFlow("UNDOCK");
        return;
      case ff_msgs::SignalState::UNPERCH:
        RunLightFlow("UNPERCH");
        return;
      case ff_msgs::SignalState::MOTION_IMPAIRED:
        RunLightFlow("MOTION_IMPAIRED");
        return;
      case ff_msgs::SignalState::THRUST_FORWARD:
        RunLightFlow("THRUST_FORWARD");
        return;
      case ff_msgs::SignalState::THRUST_AFT:
        RunLightFlow("THRUST_AFT");
        return;
      case ff_msgs::SignalState::TURN_RIGHT:
        RunLightFlow("TURN_RIGHT");
        return;
      case ff_msgs::SignalState::TURN_LEFT:
        RunLightFlow("TURN_LEFT");
        return;
      case ff_msgs::SignalState::TURN_UP:
        RunLightFlow("TURN_UP");
        return;
      case ff_msgs::SignalState::TURN_DOWN:
        RunLightFlow("TURN_DOWN");
        return;
      case ff_msgs::SignalState::CLEAR:
        RunLightFlow("CLEAR");
        return;
      case ff_msgs::SignalState::SLEEP:
        RunLightFlow("SLEEP");
        return;
      case ff_msgs::SignalState::WAKE:
        RunLightFlow("WAKE");
        return;
      case ff_msgs::SignalState::STOP_ALL_LIGHTS:
        RunLightFlow("STOP_ALL_LIGHTS");
        return;
      case ff_msgs::SignalState::CHARGING:
        RunLightFlow("CHARGING");
        return;
      default:
        ROS_ERROR("Encountered a strange state in a signal state message");
    }
  }

  void RunLightFlow(const std::string s) {
    current_state_id = s;
    auto it = light_flow_states.find(s);
    if (it == light_flow_states.end()) {
      ROS_ERROR("RunLightFlow function was given a strange string");
      return;
    }
    publishLightFlow(it->second, pub_, currentStreamingLightsState);
  }

  bool StreamingLightsCallback(
      ff_msgs::SetStreamingLights::Request& request,
      ff_msgs::SetStreamingLights::Response& response) {
    currentStreamingLightsState = request.state;
    response.success = true;
    // turn off all the lights but keep the blue streaming lights on
    RunLightFlow("STOP_ALL_LIGHTS");
    return true;
  }

 private:
  config_reader::ConfigReader config_params;  // LUA configuration reader
  std::map<std::string, Json::Value> light_flow_states;  // Available states
  ros::Subscriber sub_;                                  // Configure service
  ros::Publisher pub_;
  std::string current_state_id;
  ros::ServiceServer streaming_service_;
  bool currentStreamingLightsState;
};

PLUGINLIB_EXPORT_CLASS(light_flow::LightFlowNodelet, nodelet::Nodelet);

}  // namespace light_flow
