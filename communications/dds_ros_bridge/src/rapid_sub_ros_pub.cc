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

#include <string>

#include "dds_ros_bridge/rapid_sub_ros_pub.h"
#include "knShare/Time.h"

namespace ff {

RapidSubRosPub::RapidSubRosPub(const std::string& subscribeTopic,
                               const std::string& pubTopic,
                               const ros::NodeHandle &nh,
                               const std::string& entityName,
                               const unsigned int queueSize)
  : m_nh_(nh), m_subscribeTopic_(subscribeTopic), m_publishTopic_(pubTopic),
    m_queueSize_(queueSize), m_thread_(), m_ddsEventLoop_(entityName) {
}

RapidSubRosPub::~RapidSubRosPub() {
  m_thread_.join();
}

void RapidSubRosPub::StartThread() {
  // start joinable thread
  m_thread_ = std::thread(&RapidSubRosPub::ThreadExec, this);
}

void RapidSubRosPub::ThreadExec() {
  while (ros::ok()) {
    // process events at 10hz
    m_ddsEventLoop_.processEvents(kn::milliseconds(100));
  }
}

}  // end namespace ff
