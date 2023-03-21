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

// FSW shared libraries
#include <config_reader/config_reader.h>

// FSW nodelet
#include <ff_common/ff_names.h>
#include <ff_util/ff_component.h>

// Services
#include <ff_hw_msgs/msg/configure_led_group.hpp>
namespace ff_hw_msgs {
  typedef msg::ConfigureLEDGroup ConfigureLEDGroup;
}  // namespace ff_hw_msgs

#include <ff_msgs/srv/set_streaming_lights.hpp>
#include <ff_msgs/msg/signal_state.hpp>
namespace ff_msgs {
  typedef srv::SetStreamingLights SetStreamingLights;
  typedef msg::SignalState SignalState;
}  // namespace ff_msgs
#include <cerrno>
#include <cstring>

namespace light_flow {

FF_DEFINE_LOGGER("light_flow");

class LightFlowComponent : public ff_util::FreeFlyerComponent {
 public:
  explicit LightFlowComponent(const rclcpp::NodeOptions& options) :
  ff_util::FreeFlyerComponent(options, "light_flow", true) {}

  virtual ~LightFlowComponent() {}

 protected:
  virtual void Initialize(NodeHandle &nh) {
    // this service is a special case for when we need to light
    // two AMBER leds on each side only when we are streaming
    // live video
    streaming_service_ = FF_CREATE_SERVICE(nh, ff_msgs::SetStreamingLights, SERVICE_STREAMING_LIGHTS,
        std::bind(&LightFlowComponent::StreamingLightsCallback, this, std::placeholders::_1, std::placeholders::_2));

    currentStreamingLightsState = false;

    pub_ = FF_CREATE_PUBLISHER(nh, ff_hw_msgs::ConfigureLEDGroup,
        TOPIC_HARDWARE_SIGNAL_LIGHTS_CONFIG, 5);

    config_params.AddFile("behaviors/light_flow.config");
    if (!config_params.ReadFiles()) {
      FF_ERROR("Could not open config file");
      return;
    }

    std::string folder;
    if (!config_params.GetStr("folder", &folder)) {
      FF_ERROR("Could not get state file");
      return;
    }

    config_reader::ConfigReader::Table states(&config_params, "states");
    for (int i = 0; i < states.GetSize(); i++) {
      config_reader::ConfigReader::Table state;
      if (!states.GetTable(i + 1, &state)) {
        FF_ERROR("Could not get state table");
        return;
      }

      std::string id;
      if (!state.GetStr("id", &id)) {
        FF_ERROR("Could not get state id");
        return;
      }
      std::string file;
      if (!state.GetStr("file", &file)) {
        FF_ERROR("Could not get state file");
        return;
      }

      std::ifstream t(folder + file);
      std::stringstream document;
      document << t.rdbuf();

      Json::Reader reader;
      Json::Value compiledLightFlow;
      if (!reader.parse(document.str(), compiledLightFlow)) {
        FF_ERROR("Could not parse a lightflow json string");
        continue;
      } else {
        compileLightFlow(compiledLightFlow);
        FF_INFO("Compiled a light flow proc successfully");
      }

      light_flow_states.emplace(id, compiledLightFlow);
    }

    sub_ = FF_CREATE_SUBSCRIBER(nh, ff_msgs::SignalState, TOPIC_SIGNALS, 5,
                              std::bind(&LightFlowComponent::ConfigureCallback, this, std::placeholders::_1));
  }

  void ConfigureCallback(const std::shared_ptr<ff_msgs::SignalState> msg) {
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
        FF_ERROR("Encountered a strange state in a signal state message");
    }
  }

  void RunLightFlow(const std::string s) {
    current_state_id = s;
    auto it = light_flow_states.find(s);
    if (it == light_flow_states.end()) {
      FF_ERROR("RunLightFlow function was given a strange string");
      return;
    }
    publishLightFlow(it->second, pub_, currentStreamingLightsState);
  }

  bool StreamingLightsCallback(
      const std::shared_ptr<ff_msgs::SetStreamingLights::Request> request,
      std::shared_ptr<ff_msgs::SetStreamingLights::Response> response) {
    currentStreamingLightsState = request->state;
    response->success = true;
    // turn off all the lights but keep the amber streaming lights on
    RunLightFlow("STOP_ALL_LIGHTS");
    return true;
  }

 private:
  config_reader::ConfigReader config_params;  // LUA configuration reader
  std::map<std::string, Json::Value> light_flow_states;  // Available states

  rclcpp::Subscription<ff_msgs::SignalState>::SharedPtr sub_;                                  // Configure service
  rclcpp::Publisher<ff_hw_msgs::ConfigureLEDGroup>::SharedPtr pub_;
  std::string current_state_id;
  rclcpp::Service<ff_msgs::SetStreamingLights>::SharedPtr streaming_service_;
  bool currentStreamingLightsState;
};

}  // namespace light_flow

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(light_flow::LightFlowComponent)
