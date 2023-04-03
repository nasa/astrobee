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

// FSW nodelet
#include <ff_common/ff_names.h>
#include <ff_util/ff_component.h>


#include <ff_hw_msgs/srv/configure_system_leds.hpp>
namespace ff_hw_msgs {
typedef srv::ConfigureSystemLeds ConfigureSystemLeds;
}  // namespace ff_hw_msgs


#include <ff_msgs/srv/set_streaming_lights.hpp>
#include <ff_msgs/msg/camera_states_stamped.hpp>
#include <ff_msgs/action/dock.hpp>
#include <ff_msgs/msg/dock_state.hpp>
#include <ff_msgs/msg/ekf_state.hpp>
#include <ff_msgs/msg/heartbeat.hpp>
#include <ff_msgs/action/motion.hpp>
#include <ff_msgs/msg/motion_state.hpp>
#include <ff_msgs/msg/signal_state.hpp>
namespace ff_msgs {
typedef srv::SetStreamingLights SetStreamingLights;
typedef msg::CameraStatesStamped CameraStatesStamped;
typedef msg::DockState DockState;
typedef msg::EkfState EkfState;
typedef msg::Heartbeat Heartbeat;
typedef msg::MotionState MotionState;
typedef msg::SignalState SignalState;
}  // namespace ff_msgs

#include <sensor_msgs/msg/battery_state.hpp>
namespace sensor_msgs {
typedef msg::BatteryState BatteryState;
}  // namespace sensor_msgs

#include <cerrno>
#include <cstring>

FF_DEFINE_LOGGER("states");

namespace states {

class StatesComponent : public ff_util::FreeFlyerComponent {
 public:
  explicit StatesComponent(const rclcpp::NodeOptions& options) : ff_util::FreeFlyerComponent(options, "states", true) {}

  virtual ~StatesComponent() {
    // on nodelet shutdown, send a shutdown message on TOPIC_SIGNALS
    Shutdown();
  }

 protected:
  virtual void Initialize(NodeHandle &nh) {
    client_ = nh->create_client<ff_hw_msgs::ConfigureSystemLeds>(
        SERVICE_HARDWARE_EPS_CONF_LED_STATE);
    client_streaming_lights_ = nh->create_client<ff_msgs::SetStreamingLights>(
        SERVICE_STREAMING_LIGHTS);
    pub_ = FF_CREATE_PUBLISHER(nh, ff_msgs::SignalState, TOPIC_SIGNALS, 1);
    sub_.push_back(FF_CREATE_SUBSCRIBER(nh, ff_msgs::CameraStatesStamped, TOPIC_MANAGEMENT_CAMERA_STATE, 1,
                                 std::bind(&StatesComponent::LiveStreamingCallback, this, std::placeholders::_1)));
    // on nodelet start, send a wakeup message on TOPIC_SIGNALS
    Startup();
  }

  void Startup() {
    ff_msgs::SignalState s;
    s.state = ff_msgs::SignalState::WAKE;
    pub_->publish(s);
  }

  void Shutdown() {
    ff_msgs::SignalState s;
    s.state = ff_msgs::SignalState::SLEEP;
    pub_->publish(s);
  }

  void BatteryStateCallback(const std::shared_ptr<sensor_msgs::BatteryState> msg) {
    switch (msg->power_supply_status) {
      case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING:
        FF_INFO("We're charging!");
        ChangeCurrentState(ff_msgs::SignalState::CHARGING);
        return;
      case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_FULL:
        FF_INFO("Batteries are full!");
        return;
      default:
        return;
    }
  }

  void LiveStreamingCallback(
      const std::shared_ptr<ff_msgs::CameraStatesStamped> msg) {
    bool anyCameraStreaming = false;
    bool anyCameraRecording = false;
    for (uint i = 0; i < msg->states.size(); i++) {
      anyCameraStreaming |= msg->states[i].streaming;
      anyCameraRecording |= msg->states[i].recording;
    }

    auto systemLeds = std::make_shared<ff_hw_msgs::ConfigureSystemLeds::Request>();

    if (anyCameraStreaming) {
      systemLeds->live = ff_hw_msgs::ConfigureSystemLeds::Request::ON;
    } else {
      systemLeds->live = ff_hw_msgs::ConfigureSystemLeds::Request::OFF;
    }

    if (anyCameraRecording)
      systemLeds->video = ff_hw_msgs::ConfigureSystemLeds::Request::ON;
    else
      systemLeds->video = ff_hw_msgs::ConfigureSystemLeds::Request::OFF;

    // we no longer call the system LEDs from the states nodelet
    // as that occurs elsewhere in FSW
    // this will be cleaned up in a later commit

    auto streamingLights = std::make_shared<ff_msgs::SetStreamingLights::Request>();

    streamingLights->state = (anyCameraRecording || anyCameraStreaming);
    auto result = client_streaming_lights_->async_send_request(streamingLights);

    if (rclcpp::spin_until_future_complete(get_node_base_interface(), result) != rclcpp::FutureReturnCode::SUCCESS) {
      FF_ERROR("Failure occurred when trying to configure streaming LEDs");
    } else {
      FF_INFO("Successfully changed the streaming LEDs");
    }
  }

  void MobilityMotionStateCallback(const std::shared_ptr<ff_msgs::MotionState> msg) {
    if (msg->state == ff_msgs::MotionState::CONTROLLING) {
      ChangeCurrentState(ff_msgs::SignalState::THRUST_FORWARD);
    }
  }

  void DockStateCallback(const std::shared_ptr<ff_msgs::DockState> msg) {
    if (msg->state == ff_msgs::DockState::DOCKING_MOVING_TO_COMPLETE_POSE) {
      // we are docking!
    } else if (msg->state ==
               ff_msgs::DockState::UNDOCKING_MOVING_TO_APPROACH_POSE) {
      // we are undocking!
    }
  }
/*
  void MotionActionResultCallback(
      const rclcpp_action::ClientGoalHandle<ff_msgs::action::Motion>::WrappedResult& result) {
    if (result.result->response == 1) {
      ChangeCurrentState(ff_msgs::SignalState::SUCCESS);
    } else {
      ChangeCurrentState(ff_msgs::SignalState::MOTION_IMPAIRED);
    }
  }
*/
  void ChangeCurrentState(uint8_t new_state) {
    // if (current_state == new_state) return;
    current_state = new_state;
    ff_msgs::SignalState s;
    s.state = current_state;
    pub_->publish(s);
  }

 private:
  rclcpp::Client<ff_hw_msgs::ConfigureSystemLeds>::SharedPtr client_;
  rclcpp::Client<ff_msgs::SetStreamingLights>::SharedPtr client_streaming_lights_;
  std::vector<rclcpp::Subscription<ff_msgs::CameraStatesStamped>::SharedPtr> sub_;
  rclcpp::Publisher<ff_msgs::SignalState>::SharedPtr pub_;
  uint8_t current_state;
};


}  // namespace states

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(states::StatesComponent)

