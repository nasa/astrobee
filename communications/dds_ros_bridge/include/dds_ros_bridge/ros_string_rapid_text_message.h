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

#ifndef DDS_ROS_BRIDGE_ROS_STRING_RAPID_TEXT_MESSAGE_H_
#define DDS_ROS_BRIDGE_ROS_STRING_RAPID_TEXT_MESSAGE_H_

#include <string>

#include "dds_ros_bridge/ros_sub_rapid_pub.h"
#include "std_msgs/String.h"

#include "rapidIo/TextMessager.h"

namespace ff {

class RosStringRapidTextMessage : public RosSubRapidPub {
 public:
  RosStringRapidTextMessage(const std::string& subscribeTopic,
                            const std::string& pubTopic,
                            const ros::NodeHandle &nh,
                            const unsigned int queueSize = 10,
                            const std::string& category = "ff");

  void CallBack(const std_msgs::String::ConstPtr& msg);

 private:
  std::string m_category_;
  rapid::TextMessagerParameters m_params_;
  std::shared_ptr<rapid::TextMessager> m_provider_;
};

}  // end namespace ff

#endif  // DDS_ROS_BRIDGE_ROS_STRING_RAPID_TEXT_MESSAGE_H_
