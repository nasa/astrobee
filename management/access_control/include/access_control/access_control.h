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

#include <ff_common/ff_names.h>
#include <ff_util/ff_component.h>

#include <config_reader/config_reader.h>

#include <ff_msgs/msg/access_control_state_stamped.hpp>
#include <ff_msgs/msg/ack_completed_status.hpp>
#include <ff_msgs/msg/ack_stamped.hpp>
#include <ff_msgs/msg/ack_status.hpp>
#include <ff_msgs/msg/command_constants.hpp>
#include <ff_msgs/msg/command_stamped.hpp>

namespace ff_msgs {
  typedef msg::AccessControlStateStamped AccessControlStateStamped;
  typedef msg::AckCompletedStatus AckCompletedStatus;
  typedef msg::AckStamped AckStamped;
  typedef msg::AckStatus AckStatus;
  typedef msg::CommandArg CommandArg;
  typedef msg::CommandConstants CommandConstants;
  typedef msg::CommandStamped CommandStamped;
}  // namespace ff_msgs

#include <cstdint>
#include <iostream>
#include <random>
#include <string>

namespace access_control {

class AccessControl : public ff_util::FreeFlyerComponent {
 public:
  explicit AccessControl(const rclcpp::NodeOptions & options);
  ~AccessControl();

 protected:
  virtual void Initialize(NodeHandle &nh);

 private:
  std::string GenerateCookie();

  void HandleCommand(ff_msgs::CommandStamped const& cmd);

  void HandleGrabControl(ff_msgs::CommandStamped const& cmd);

  void HandleRequestControl(ff_msgs::CommandStamped const& cmd);

  void PublishAck(std::string const& cmd_id,
                  std::string const& message = "",
                  uint8_t completed_status = ff_msgs::AckCompletedStatus::OK,
                  uint8_t status = ff_msgs::AckStatus::COMPLETED);

  void PublishCommand(ff_msgs::CommandStamped const& cmd);

  void PublishState();

  ff_msgs::AccessControlStateStamped state_;
  ff_msgs::AckStamped ack_;

  rclcpp::Node::SharedPtr node_;

  int pub_queue_size_;
  int sub_queue_size_;

  Publisher<ff_msgs::AckStamped> cmd_ack_pub_;
  Publisher<ff_msgs::AccessControlStateStamped> state_pub_;
  Publisher<ff_msgs::CommandStamped> cmd_pub_;
  Publisher<ff_msgs::CommandStamped> failed_cmd_pub_;
  Subscriber<ff_msgs::CommandStamped> cmd_sub_;

  std::string requestor_;
};

}  // namespace access_control

#endif  // ACCESS_CONTROL_ACCESS_CONTROL_H_
