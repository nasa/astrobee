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

#include "dds_ros_bridge/ros_payload_state.h"

ff::RosPayloadStateToRapid::RosPayloadStateToRapid(
                                            const std::string& subscribe_topic,
                                            const std::string& pub_topic,
                                            const ros::NodeHandle &nh,
                                            const unsigned int queue_size)
  : RosSubRapidPub(subscribe_topic, pub_topic, nh, queue_size) {
  state_supplier_.reset(new ff::RosPayloadStateToRapid::StateSupplier(
       rapid::ext::astrobee::PAYLOAD_STATE_TOPIC + pub_topic, "",
       "AstrobeePayloadStateProfile", ""));

  config_supplier_.reset(new ff::RosPayloadStateToRapid::ConfigSupplier(
        rapid::ext::astrobee::PAYLOAD_CONFIG_TOPIC + pub_topic, "",
        "AstrobeePayloadConfigProfile", ""));

  sub_ = nh_.subscribe(subscribe_topic,
                       queue_size,
                       &RosPayloadStateToRapid::Callback,
                       this);

  rapid::ext::astrobee::PayloadConfig &config_msg = config_supplier_->event();
  rapid::ext::astrobee::PayloadState &state_msg = state_supplier_->event();

  rapid::RapidHelper::initHeader(config_msg.hdr);
  rapid::RapidHelper::initHeader(state_msg.hdr);

  // Initialize the serial number to be 0. Technically this could have been set
  // in the initHeader function but it is the last argument and I didn't care to
  // specify any of the other arguments
  config_msg.hdr.serial = 0;
  state_msg.hdr.serial = 0;

  // Resize the arrays
  config_msg.payloadNames.length(4);
  state_msg.powered.length(4);

  // Fill out config message
  std::strncpy(config_msg.payloadNames[0], "Top Front", 9);
  config_msg.payloadNames[0][9] = '\0';

  std::strncpy(config_msg.payloadNames[1], "Top Aft", 7);
  config_msg.payloadNames[1][7] = '\0';

  std::strncpy(config_msg.payloadNames[2], "Bottom Front", 12);
  config_msg.payloadNames[2][12] = '\0';

  std::strncpy(config_msg.payloadNames[3], "Bottom Aft", 10);
  config_msg.payloadNames[3][10] = '\0';

  // Fill out the initial state message
  state_msg.powered[0] = false;
  state_msg.powered[1] = false;
  state_msg.powered[2] = false;
  state_msg.powered[3] = false;

  // Send config and state messages
  config_msg.hdr.timeStamp = util::RosTime2RapidTime(ros::Time::now());
  config_supplier_->sendEvent();

  state_msg.hdr.timeStamp = util::RosTime2RapidTime(ros::Time::now());
  state_supplier_->sendEvent();
}


void ff::RosPayloadStateToRapid::Callback(
                              ff_hw_msgs::EpsPowerStateConstPtr const& state) {
  bool publish = false;
  unsigned int i;

  rapid::ext::astrobee::PayloadState &state_msg = state_supplier_->event();

  for (i = 0; i < state->values.size(); i++) {
    // Find and check if top front power has changed
    if (state->values[i].name == "PAYLOAD_EN_TOP_FRONT") {
      if (state->values[i].value != state_msg.powered[0]) {
        state_msg.powered[0] = state->values[i].value;
        publish = true;
      }
    }

    // Find and check if top aft power has changed
    if (state->values[i].name == "PAYLOAD_EN_TOP_AFT") {
      if (state->values[i].value != state_msg.powered[1]) {
        state_msg.powered[1] = state->values[i].value;
        publish = true;
      }
    }

    // Find and check if bottom front power has changed
    if (state->values[i].name == "PAYLOAD_EN_BOT_FRONT") {
      if (state->values[i].value != state_msg.powered[2]) {
        state_msg.powered[2] = state->values[i].value;
        publish = true;
      }
    }

    // Find and check if bottom aft power has changed
    if (state->values[i].name == "PAYLOAD_EN_BOT_AFT") {
      if (state->values[i].value != state_msg.powered[3]) {
        state_msg.powered[3] = state->values[i].value;
        publish = true;
      }
    }
  }

  if (publish) {
    state_msg.hdr.timeStamp = util::RosTime2RapidTime(ros::Time::now());
    state_supplier_->sendEvent();
  }
}
