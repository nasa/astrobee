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

#include "dds_ros_bridge/rapid_sub_ros_pub.h"

namespace ff {

RapidSubRosPub::RapidSubRosPub(const std::string& subscribe_topic,
                               const std::string& pub_topic,
                               const ros::NodeHandle &nh,
                               const std::string& entity_name,
                               const unsigned int queue_size)
  : nh_(nh), subscribe_topic_(subscribe_topic), publish_topic_(pub_topic),
    queue_size_(queue_size), thread_(), dds_event_loop_(entity_name) {
}

RapidSubRosPub::~RapidSubRosPub() {
  thread_.join();
}

void RapidSubRosPub::StartThread() {
  // start joinable thread
  thread_ = std::thread(&RapidSubRosPub::ThreadExec, this);
}

void RapidSubRosPub::ThreadExec() {
  while (ros::ok()) {
    // process events at 10hz
    dds_event_loop_.processEvents(kn::milliseconds(100));
  }
}

}  // end namespace ff
