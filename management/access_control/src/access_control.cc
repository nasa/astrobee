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

#include <access_control/access_control.h>

namespace access_control {

constexpr char hex[] = "0123456789abcdef";

static inline void byte2hex(const std::uint8_t b, char dest[2]) {
  dest[0] = hex[b >> 4];
  dest[1] = hex[b & 0x0f];
}

AccessControl::AccessControl() :
    ff_util::FreeFlyerNodelet(NODE_ACCESS_CONTROL, true),
    pub_queue_size_(10),
    sub_queue_size_(10) {
}

AccessControl::~AccessControl() {
}

void AccessControl::Initialize(ros::NodeHandle *nh) {
  state_.header.frame_id = "world";
  ack_.header.frame_id = "world";

  cmd_sub_ = nh->subscribe<ff_msgs::CommandStamped>(
                                            TOPIC_COMMUNICATIONS_DDS_COMMAND,
                                            sub_queue_size_,
                                            &AccessControl::HandleCommand,
                                            this);

  cmd_ack_pub_ = nh->advertise<ff_msgs::AckStamped>(TOPIC_MANAGEMENT_ACK,
                                                    pub_queue_size_, false);

  // All states should be latching
  state_pub_ = nh->advertise<ff_msgs::AccessControlStateStamped>(
                                          TOPIC_MANAGEMENT_ACCESS_CONTROL_STATE,
                                          pub_queue_size_,
                                          true);

  cmd_pub_ = nh->advertise<ff_msgs::CommandStamped>(TOPIC_COMMAND,
                                                    pub_queue_size_, false);

  failed_cmd_pub_ = nh->advertise<ff_msgs::CommandStamped>(
                                            TOPIC_MANAGEMENT_ACCESS_CONTROL_CMD,
                                            pub_queue_size_,
                                            true);

  state_.controller = "No Controller";

  // Publish initial state
  PublishState();
}

std::string AccessControl::GenerateCookie() {
  static std::random_device rd;
  static std::mt19937 mt(static_cast<uint32_t>(rd()));

  char data[32] = {};
  std::string s(32, '0');

  char * p = data;

  for (int i = 0; i < 2; i++) {
    uint32_t val = mt();
    for (int j = 0; j < 4; j++) {
      byte2hex(val & 0xFF, p);
      p += 2;
      val = val >> 8;
    }
  }

  s.assign(data);
  return s;
}

void AccessControl::HandleCommand(ff_msgs::CommandStampedConstPtr const& cmd) {
  if (cmd->cmd_name == ff_msgs::CommandConstants::CMD_NAME_REQUEST_CONTROL) {
    HandleRequestControl(cmd);
  } else if (cmd->cmd_name == ff_msgs::CommandConstants::CMD_NAME_GRAB_CONTROL) {
    HandleGrabControl(cmd);
  } else if ((state_.controller != "" && state_.controller == cmd->cmd_src) ||
        cmd->cmd_name == ff_msgs::CommandConstants::CMD_NAME_IDLE_PROPULSION ||
        cmd->cmd_name == ff_msgs::CommandConstants::CMD_NAME_STOP_ALL_MOTION) {
    // Always let idle propulsion, stop, and commands sent by the operator in
    // control through. Also make sure someone has grabbed control before
    // letting commands other than stop and idle through.
    PublishCommand(cmd);
  } else {  // Operator isn't in control and cannot issue commands
    PublishAck(cmd->cmd_id,
               "Operator doesn't have control.",
               ff_msgs::AckCompletedStatus::BAD_SYNTAX);
    // Need to publish the command so that it gets echoed to the ground
    failed_cmd_pub_.publish(cmd);
  }
}

void AccessControl::HandleGrabControl(ff_msgs::CommandStampedConstPtr const&
                                                                          cmd) {
  std::string msg;
  if (cmd->args.size() != 1) {
    msg = "Invalid number of arguments to grab control command.";
    ROS_WARN("%s", msg.c_str());
    PublishAck(cmd->cmd_id,
               msg,
               ff_msgs::AckCompletedStatus::BAD_SYNTAX);
    return;
  }

  if (cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING) {
    msg = "Invalid argument type to grab control command.";
    ROS_WARN("%s", msg.c_str());
    PublishAck(cmd->cmd_id,
               msg,
               ff_msgs::AckCompletedStatus::BAD_SYNTAX);
    return;
  }

  const std::string cookie = cmd->args[0].s;
  if (cookie != state_.cookie) {
    msg = "Agent " + cmd->cmd_src + " sent wrong cookie. Expected "
          + state_.cookie + ", got " + cookie;
    ROS_WARN("%s", msg.c_str());
    PublishAck(cmd->cmd_id,
               msg,
               ff_msgs::AckCompletedStatus::EXEC_FAILED);
    return;
  }

  if (cmd->cmd_src != requestor_) {
    ROS_WARN("Agent %s has grabbed control, yet %s requested it",
             cmd->cmd_src.data(), requestor_.data());
  }

  state_.controller = cmd->cmd_src;
  state_.cookie = "";

  PublishState();
  PublishAck(cmd->cmd_id);
  return;
}

void AccessControl::HandleRequestControl(ff_msgs::CommandStampedConstPtr const&
                                                                          cmd) {
  state_.cookie = GenerateCookie();
  requestor_ = cmd->cmd_src;

  PublishAck(cmd->cmd_id);
  PublishState();
}

void AccessControl::PublishAck(std::string const& cmd_id,
                               std::string const& message,
                               uint8_t completed_status,
                               uint8_t status) {
  ack_.header.stamp = ros::Time::now();
  ack_.cmd_id = cmd_id;
  ack_.status.status = status;
  ack_.completed_status.status = completed_status;
  ack_.message = message;
  cmd_ack_pub_.publish(ack_);
}

void AccessControl::PublishCommand(ff_msgs::CommandStampedConstPtr const& cmd) {
  cmd_pub_.publish(cmd);
}

void AccessControl::PublishState() {
  state_.header.stamp = ros::Time::now();
  state_pub_.publish(state_);
}

}  // namespace access_control

PLUGINLIB_EXPORT_CLASS(access_control::AccessControl, nodelet::Nodelet)
