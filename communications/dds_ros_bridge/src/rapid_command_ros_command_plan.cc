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

#include "dds_ros_bridge/rapid_command_ros_command_plan.h"

using ff_msgs::CommandStamped;
using ff_msgs::CommandArg;

namespace {

void TransferArgument(rapid::ParameterUnion const& u,
                      ff_msgs::CommandArg *arg) {
  const rapid::DataType type = u._d;
  switch (type) {
  case rapid::RAPID_BOOL:
    arg->data_type = CommandArg::DATA_TYPE_BOOL;
    arg->b = u._u.b;
    break;
  case rapid::RAPID_DOUBLE:
    arg->data_type = CommandArg::DATA_TYPE_DOUBLE;
    arg->d = u._u.d;
    break;
  case rapid::RAPID_FLOAT:
    arg->data_type = CommandArg::DATA_TYPE_FLOAT;
    arg->f = u._u.f;
    break;
  case rapid::RAPID_INT:
    arg->data_type = CommandArg::DATA_TYPE_INT;
    arg->i = u._u.i;
    break;
  case rapid::RAPID_LONGLONG:
    arg->data_type = CommandArg::DATA_TYPE_LONGLONG;
    arg->ll = u._u.ll;
    break;
  case rapid::RAPID_STRING:
    arg->data_type = CommandArg::DATA_TYPE_STRING;
    arg->s = u._u.s;
    break;
  case rapid::RAPID_VEC3d:
    arg->data_type = CommandArg::DATA_TYPE_VEC3d;
    for (int i = 0; i < 3; i++) {
      arg->vec3d[i] = u._u.vec3d[i];
    }
    break;
  case rapid::RAPID_MAT33f:
    arg->data_type = CommandArg::DATA_TYPE_MAT33f;
    for (int i = 0; i < 9; i++) {
      arg->mat33f[i] = u._u.mat33f[i];
    }
    break;
  default:
    ROS_ASSERT_MSG(false, "unknown rapid::DATA_TYPE: %d", type);
  }
}

}  // end namespace

namespace ff {

RapidCommandRosCommand::RapidCommandRosCommand(
                                            const std::string& subscribe_topic,
                                            const std::string& pub_topic,
                                            const ros::NodeHandle &nh,
                                            const unsigned int queue_size)
  : RapidSubRosPub(subscribe_topic,
                   pub_topic,
                   nh,
                   "RapidCommandRosCommand",
                   queue_size) {
  // advertise ros topic
  pub_ = nh_.advertise<ff_msgs::CommandStamped>(pub_topic, queue_size);

  // connect to ddsEventLoop
  // @todo confirm topic suffix has '-'
  try {
    dds_event_loop_.connect<rapid::Command>(this,
                                            rapid::COMMAND_TOPIC +
                                            subscribe_topic,        // topic
                                            "",                     // name
                                            "RapidCommandProfile",  // profile
                                            "");                    // library
  } catch (std::exception& e) {
    ROS_ERROR_STREAM("RapidCommandRosCommand exception: " << e.what());
    throw;
  } catch (...) {
    ROS_ERROR("RapidCommandRosCommand exception unknown");
    throw;
  }

  // start thread
  StartThread();
}

void RapidCommandRosCommand::operator() (rapid::Command const* rapid_cmd) {
  // TODO(tfmorse): Validate command against commandConfig message

  ff_msgs::CommandStamped cmd;
  util::RapidHeader2Ros(rapid_cmd->hdr, &cmd.header);

  cmd.cmd_name = rapid_cmd->cmdName;
  cmd.cmd_id = rapid_cmd->cmdId;
  cmd.cmd_src = rapid_cmd->cmdSrc;
  cmd.cmd_origin = "ground";
  cmd.subsys_name = rapid_cmd->subsysName;

  cmd.args.resize(rapid_cmd->arguments.length());
  for (int i = 0; i < rapid_cmd->arguments.length(); ++i) {
    TransferArgument(rapid_cmd->arguments[i], &cmd.args[i]);
  }

  pub_.publish(cmd);
}

}  // end namespace ff
