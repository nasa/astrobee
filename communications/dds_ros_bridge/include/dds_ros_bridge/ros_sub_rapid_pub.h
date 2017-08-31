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


#ifndef DDS_ROS_BRIDGE_ROS_SUB_RAPID_PUB_H_
#define DDS_ROS_BRIDGE_ROS_SUB_RAPID_PUB_H_

#include <memory>
#include <string>

#include "ros/ros.h"

namespace ff {

class RosSubRapidPub {
 protected:
  RosSubRapidPub(const std::string& subscribeTopic,
      const std::string& pubTopic, const ros::NodeHandle &nh,
      const unsigned int queueSize)
    : m_nh_(nh), m_subscribeTopic_(subscribeTopic), m_publishTopic_(pubTopic),
      m_queueSize_(queueSize) {
  }

  ros::NodeHandle m_nh_;
  ros::Subscriber m_sub_;
  std::string m_subscribeTopic_;
  std::string m_publishTopic_;
  unsigned int m_queueSize_;
};

typedef std::shared_ptr<ff::RosSubRapidPub> RosSubRapidPubPtr;

}  // end namespace ff

#endif  // DDS_ROS_BRIDGE_ROS_SUB_RAPID_PUB_H_
