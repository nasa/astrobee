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


#ifndef GROUND_DDS_ROS_BRIDGE_ROS_SUB_RAPID_PUB_H_
#define GROUND_DDS_ROS_BRIDGE_ROS_SUB_RAPID_PUB_H_

#include <memory>
#include <string>

#include "ros/ros.h"

namespace ff {

class RosSubRapidPub {
 protected:
  RosSubRapidPub(const std::string& subscribe_topic,
                 const std::string& pub_topic,
                 const ros::NodeHandle &nh,
                 const unsigned int queue_size)
    : nh_(nh),
      subscribe_topic_(subscribe_topic),
      publish_topic_(pub_topic),
      queue_size_(queue_size) {
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  std::string subscribe_topic_;
  std::string publish_topic_;
  unsigned int queue_size_;
};

typedef std::shared_ptr<ff::RosSubRapidPub> RosSubRapidPubPtr;

}  // end namespace ff

#endif  // GROUND_DDS_ROS_BRIDGE_ROS_SUB_RAPID_PUB_H_
