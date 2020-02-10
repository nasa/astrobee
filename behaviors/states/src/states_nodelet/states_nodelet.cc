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
#include <ff_msgs/SetStreamingLights.h>

#include <ff_hw_msgs/ConfigureSystemLeds.h>
#include <ff_msgs/CameraStatesStamped.h>
#include <ff_msgs/DockActionResult.h>
#include <ff_msgs/DockState.h>
#include <ff_msgs/EkfState.h>
#include <ff_msgs/Heartbeat.h>
#include <ff_msgs/MotionActionResult.h>
#include <ff_msgs/MotionState.h>
#include <ff_msgs/SignalState.h>
#include <sensor_msgs/BatteryState.h>

#include <cerrno>
#include <cstring>

namespace states {

class StatesNodelet : public ff_util::FreeFlyerNodelet {
 public:
  StatesNodelet() : ff_util::FreeFlyerNodelet("states") {}

  virtual ~StatesNodelet() {
    // on nodelet shutdown, send a shutdown message on TOPIC_SIGNALS
    Shutdown();
  }

 protected:
  virtual void Initialize(ros::NodeHandle* nh) {
    client_ = nh->serviceClient<ff_hw_msgs::ConfigureSystemLeds>(
        SERVICE_HARDWARE_EPS_CONF_LED_STATE);
    client_streaming_lights_ = nh->serviceClient<ff_msgs::SetStreamingLights>(
        SERVICE_STREAMING_LIGHTS);
    pub_ = nh->advertise<ff_msgs::SignalState>(
        TOPIC_SIGNALS, 1, true);  // TOPIC_SIGNALS is a latched topic
    sub_.push_back(nh->subscribe(TOPIC_MANAGEMENT_CAMERA_STATE, 1,
                                 &StatesNodelet::LiveStreamingCallback, this));
    // on nodelet start, send a wakeup message on TOPIC_SIGNALS
    Startup();
  }

  void Startup() {
    ff_msgs::SignalState s;
    s.state = ff_msgs::SignalState::WAKE;
    pub_.publish(s);
  }

  void Shutdown() {
    ff_msgs::SignalState s;
    s.state = ff_msgs::SignalState::SLEEP;
    pub_.publish(s);
  }

  void BatteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& msg) {
    switch (msg->power_supply_status) {
      case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING:
        ROS_INFO("We're charging!");
        ChangeCurrentState(ff_msgs::SignalState::CHARGING);
        return;
      case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_FULL:
        ROS_INFO("Batteries are full!");
        return;
      default:
        return;
    }
  }

  void LiveStreamingCallback(
      const ff_msgs::CameraStatesStamped::ConstPtr& msg) {
    bool anyCameraStreaming = false;
    bool anyCameraRecording = false;
    for (uint i = 0; i < msg->states.size(); i++) {
      anyCameraStreaming |= msg->states[i].streaming;
      anyCameraRecording |= msg->states[i].recording;
    }

    ff_hw_msgs::ConfigureSystemLeds systemLeds;

    if (anyCameraStreaming) {
      systemLeds.request.live = systemLeds.request.ON;
    } else {
      systemLeds.request.live = systemLeds.request.OFF;
    }

    if (anyCameraRecording)
      systemLeds.request.video = systemLeds.request.ON;
    else
      systemLeds.request.video = systemLeds.request.OFF;

    // we no longer call the system LEDs from the states nodelet
    // as that occurs elsewhere in FSW
    // this will be cleaned up in a later commit

    ff_msgs::SetStreamingLights streamingLights;

    streamingLights.request.state = (anyCameraRecording || anyCameraStreaming);
    if (!client_streaming_lights_.call(streamingLights)) {
      ROS_ERROR("Failiure occurred when trying to configure streaming LEDs");
    } else {
      ROS_INFO("Successfully changed the streaming LEDs");
    }
  }

  void MobilityMotionStateCallback(const ff_msgs::MotionState::ConstPtr& msg) {
    if (msg->state == ff_msgs::MotionState::CONTROLLING) {
      ChangeCurrentState(ff_msgs::SignalState::THRUST_FORWARD);
    }
  }

  void DockStateCallback(const ff_msgs::DockState::ConstPtr& msg) {
    if (msg->state == ff_msgs::DockState::DOCKING_MOVING_TO_COMPLETE_POSE) {
      // we are docking!
    } else if (msg->state ==
               ff_msgs::DockState::UNDOCKING_MOVING_TO_APPROACH_POSE) {
      // we are undocking!
    }
  }

  void MotionActionResultCallback(
      const ff_msgs::MotionActionResult::ConstPtr& msg) {
    if (msg->result.response == 1) {
      ChangeCurrentState(ff_msgs::SignalState::SUCCESS);
    } else {
      ChangeCurrentState(ff_msgs::SignalState::MOTION_IMPAIRED);
    }
  }

  void ChangeCurrentState(uint8_t new_state) {
    // if (current_state == new_state) return;
    current_state = new_state;
    ff_msgs::SignalState s;
    s.state = current_state;
    pub_.publish(s);
  }

 private:
  ros::ServiceClient client_;
  ros::ServiceClient client_streaming_lights_;
  std::vector<ros::Subscriber> sub_;
  ros::Publisher pub_;
  uint8_t current_state;
};

PLUGINLIB_EXPORT_CLASS(states::StatesNodelet, nodelet::Nodelet);

}  // namespace states
