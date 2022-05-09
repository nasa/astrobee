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

#include "ground_dds_ros_bridge/rapid_access_control_state.h"

namespace ff {

RapidAccessControlStateToRos::RapidAccessControlStateToRos(
                                            const std::string& subscribe_topic,
                                            const std::string& pub_topic,
                                            const ros::NodeHandle &nh,
                                            const unsigned int queue_size)
  : RapidSubRosPub(subscribe_topic,
                   pub_topic,
                   nh,
                   "RapidAccessControlStateToRos",
                   queue_size) {
  // advertise ros topic, make it latched
  pub_ = nh_.advertise<ff_msgs::AccessControlStateStamped>(publish_topic_,
                                                           queue_size,
                                                           true);

  // connect to ddsEventLoop
  try {
    dds_event_loop_.connect<rapid::AccessControlState>(this,
                  rapid::ACCESSCONTROL_STATE_TOPIC + subscribe_topic,  // topic
                  "",                                                  // name
                  "RapidAccessControlStateProfile",                    // profile
                  "");                                                 // library
  } catch (std::exception& e) {
    ROS_ERROR_STREAM("RapidAccessControlStateToRos exception: " << e.what());
    throw;
  } catch (...) {
    ROS_ERROR("RapidAccessControlStateToRos exception unknown");
    throw;
  }

  // start thread
  StartThread();
}

void RapidAccessControlStateToRos::operator() (
                  rapid::AccessControlState const* rapid_access_control_state) {
  ff_msgs::AccessControlStateStamped state;
  util::RapidHeader2Ros(rapid_access_control_state->hdr, &state.header);

  state.controller = rapid_access_control_state->controller;

  // Astrobee uses the requestors field for the cookie. Thus we only need the
  // first element.
  state.cookie = rapid_access_control_state->requestors[0];

  pub_.publish(state);
}

}  // end namespace ff
