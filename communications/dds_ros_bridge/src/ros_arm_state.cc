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

#include <ros/assert.h>

#include <string>
#include <cstring>
#include <memory>

#include "dds_ros_bridge/ros_arm_state.h"
#include "dds_ros_bridge/util.h"

#include "rapidUtil/RapidHelper.h"

#include "ff_msgs/ArmStateStamped.h"
#include "ArmStateSupport.h"

namespace rea = rapid::ext::astrobee;

using ff_msgs::ArmStateStamped;
using ff_msgs::ArmJointState;
using ff_msgs::ArmGripperState;
using rea::ArmState;

namespace {

#define GENERATE_JOINT_CASE(NAME) \
  case ArmJointState::NAME: return rea::ARM_JOINT_STATE_##NAME

#define GENERATE_JOINT_DEFAULT(NAME) \
  default: \
    ROS_FATAL("unknown %s: %d", state_name, state); \
    return rea::ARM_JOINT_STATE_##NAME

#define GENERATE_GRIPPER_CASE(NAME) \
  case ArmGripperState::NAME: return rea::ARM_GRIPPER_STATE_##NAME

#define GENERATE_GRIPPER_DEFAULT(NAME) \
  default: \
    ROS_FATAL("unknown %s: %d", state_name, state); \
    return rea::ARM_GRIPPER_STATE_##NAME

rea::ArmJointState ConvertJointState(const uint8_t state) {
  static const char* state_name = "arm joint state";
  switch (state) {
    GENERATE_JOINT_CASE(UNKNOWN);
    GENERATE_JOINT_CASE(STOWED);
    GENERATE_JOINT_CASE(DEPLOYING);
    GENERATE_JOINT_CASE(STOPPED);
    GENERATE_JOINT_CASE(MOVING);
    GENERATE_JOINT_CASE(STOWING);
    GENERATE_JOINT_DEFAULT(UNKNOWN);
  }
}

rea::ArmGripperState ConvertGripperState(const uint8_t state) {
  static const char* state_name = "arm gripper state";
  switch (state) {
    GENERATE_GRIPPER_CASE(UNKNOWN);
    GENERATE_GRIPPER_CASE(UNCALIBRATED);
    GENERATE_GRIPPER_CASE(CALIBRATING);
    GENERATE_GRIPPER_CASE(CLOSED);
    GENERATE_GRIPPER_CASE(OPEN);
    GENERATE_GRIPPER_DEFAULT(UNKNOWN);
  }
}

}  // end namespace

ff::RosArmStateToRapid::RosArmStateToRapid(
    const std::string& subscribeTopic,
    const std::string& pubTopic,
    const ros::NodeHandle &nh,
    const unsigned int queueSize)
  : RosSubRapidPub(subscribeTopic, pubTopic, nh, queueSize) {
  m_supplier_.reset(
    new ff::RosArmStateToRapid::StateSupplier(
        rapid::ext::astrobee::ARM_STATE_TOPIC + pubTopic,
      "", "AstrobeeArmState", ""));

  m_sub_ = m_nh_.subscribe(subscribeTopic, queueSize,
    &RosArmStateToRapid::Callback, this);

  rapid::RapidHelper::initHeader(m_supplier_->event().hdr);
}

void ff::RosArmStateToRapid::Callback(
  const ff_msgs::ArmStateStamped::ConstPtr& status) {
  rea::ArmState &msg = m_supplier_->event();

  msg.hdr.timeStamp = util::RosTime2RapidTime(status->header.stamp);

  msg.jointState = ConvertJointState(status->joint_state.state);
  msg.gripperState = ConvertGripperState(status->gripper_state.state);

  m_supplier_->sendEvent();
}
