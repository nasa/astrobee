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

#ifndef ACCESS_CONTROL_ACCESS_CONTROL_H_
#define ACCESS_CONTROL_ACCESS_CONTROL_H_

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <config_reader/config_reader.h>
#include <ff_msgs/AccessControlStateStamped.h>
#include <ff_msgs/AckCompletedStatus.h>
#include <ff_msgs/AckStamped.h>
#include <ff_msgs/AckStatus.h>
#include <ff_msgs/CommandConstants.h>
#include <ff_msgs/CommandStamped.h>
#include <ff_util/ff_names.h>
#include <ff_util/ff_nodelet.h>

#include <cstdint>
#include <iostream>
#include <random>
#include <string>

namespace access_control {

class AccessControl : public ff_util::FreeFlyerNodelet {
 public:
  AccessControl();
  ~AccessControl();

 protected:
  virtual void Initialize(ros::NodeHandle *nh);

 private:
  std::string GenerateCookie();

  void HandleCommand(ff_msgs::CommandStampedConstPtr const& cmd);

  void HandleGrabControl(ff_msgs::CommandStampedConstPtr const& cmd);

  void HandleRequestControl(ff_msgs::CommandStampedConstPtr const& cmd);

  void PublishAck(std::string const& cmd_id,
                  std::string const& cmd_origin,
                  std::string const& message = "",
                  uint8_t completed_status = ff_msgs::AckCompletedStatus::OK,
                  uint8_t status = ff_msgs::AckStatus::COMPLETED);

  void PublishCommand(ff_msgs::CommandStampedConstPtr const& cmd);

  void PublishState();

  ff_msgs::AccessControlStateStamped state_;
  ff_msgs::AckStamped ack_;

  int pub_queue_size_;
  int sub_queue_size_;

  ros::Publisher cmd_ack_pub_, state_pub_, cmd_pub_;
  ros::Subscriber cmd_sub_;

  std::string requestor_;
};

}  // namespace access_control

#endif  // ACCESS_CONTROL_ACCESS_CONTROL_H_
