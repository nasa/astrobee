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

#include "ground_dds_ros_bridge/ros_command.h"

ff::RosCommandToRapid::RosCommandToRapid(const std::string& subscribe_topic,
                                         const std::string& pub_topic,
                                         const std::string& connecting_robot,
                                         const ros::NodeHandle &nh,
                                         const unsigned int queue_size)
  : RosSubRapidPub(subscribe_topic, pub_topic, nh, queue_size) {
  command_supplier_.reset(
    new ff::RosCommandToRapid::CommandSupplier(
      rapid::COMMAND_TOPIC + pub_topic, "", "RapidCommandProfile", ""));

  sub_ = nh_.subscribe(subscribe_topic,
                       queue_size,
                       &RosCommandToRapid::CmdCallback,
                       this);

  rapid::RapidHelper::initHeader(command_supplier_->event().hdr,
                                 "",
                                 connecting_robot);
}

void ff::RosCommandToRapid::CmdCallback(
                                  ff_msgs::CommandStampedConstPtr const& cmd) {
  ROS_ERROR("Received a ros command.");
  rapid::Command &msg = command_supplier_->event();

  msg.hdr.timeStamp = util::RosTime2RapidTime(cmd->header.stamp);

  std::strncpy(msg.cmdName, cmd->cmd_name.data(), 64);
  msg.cmdName[63] = '\0';

  std::strncpy(msg.cmdId, cmd->cmd_id.data(), 64);
  msg.cmdName[63] = '\0';

  std::strncpy(msg.cmdSrc, cmd->cmd_src.data(), 32);
  msg.cmdSrc[31] = '\0';

  std::strncpy(msg.subsysName, cmd->subsys_name.data(), 32);
  msg.subsysName[31] = '\0';

  msg.arguments.length(cmd->args.size());
  for (unsigned int i = 0; i < cmd->args.size(); i++) {
    switch (cmd->args[i].data_type) {
      case ff_msgs::CommandArg::DATA_TYPE_BOOL:
        msg.arguments[i]._d = rapid::RAPID_BOOL;
        msg.arguments[i]._u.b = cmd->args[i].b;
        break;
      case ff_msgs::CommandArg::DATA_TYPE_DOUBLE:
        msg.arguments[i]._d = rapid::RAPID_DOUBLE;
        msg.arguments[i]._u.d = cmd->args[i].d;
        break;
      case ff_msgs::CommandArg::DATA_TYPE_FLOAT:
        msg.arguments[i]._d = rapid::RAPID_FLOAT;
        msg.arguments[i]._u.f = cmd->args[i].f;
        break;
      case ff_msgs::CommandArg::DATA_TYPE_INT:
        msg.arguments[i]._d = rapid::RAPID_INT;
        msg.arguments[i]._u.i = cmd->args[i].i;
        break;
      case ff_msgs::CommandArg::DATA_TYPE_LONGLONG:
        msg.arguments[i]._d = rapid::RAPID_LONGLONG;
        msg.arguments[i]._u.ll = cmd->args[i].ll;
        break;
      case ff_msgs::CommandArg::DATA_TYPE_STRING:
        msg.arguments[i]._d = rapid::RAPID_STRING;
        std::strncpy(msg.arguments[i]._u.s, cmd->args[i].s.data(), 128);
        msg.arguments[i]._u.s[127] = '\0';
        break;
      case ff_msgs::CommandArg::DATA_TYPE_VEC3d:
        msg.arguments[i]._d = rapid::RAPID_VEC3d;
        for (int j = 0; j < 3; j++) {
          msg.arguments[i]._u.vec3d[j] = cmd->args[i].vec3d[j];
        }
        break;
      case ff_msgs::CommandArg::DATA_TYPE_MAT33f:
        msg.arguments[i]._d = rapid::RAPID_MAT33f;
        for (int j = 0; j < 9; j++) {
          msg.arguments[i]._u.mat33f[j] = cmd->args[i].mat33f[j];
        }
        break;
      default:
        ROS_ERROR("Ground DDS bridge: Unknown command data type: %d",
                                                        cmd->args[i].data_type);
      }
    }

  ROS_ERROR("Sending dds command.");
  command_supplier_->sendEvent();
}
