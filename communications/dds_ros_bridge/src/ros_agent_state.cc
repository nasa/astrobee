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

#include "dds_ros_bridge/ros_agent_state.h"

namespace rea = rapid::ext::astrobee;

using ff_msgs::AgentStateStamped;
using ff_msgs::OpState;
using ff_msgs::ExecState;
using ff_msgs::MobilityState;
using rea::AgentState;

namespace {

#define GENERATE_OP_CASE(NAME) \
  case OpState::NAME: return rea::OPERATING_STATE_##NAME

#define GENERATE_OP_DEFAULT(NAME) \
  default: \
    ROS_FATAL("unknown %s: %d", state_name, state); \
    return rea::OPERATING_STATE_##NAME


rea::OperatingState ConvertOperatingState(const uint8_t state) {
  static const char* state_name = "operating state";
  switch (state) {
    GENERATE_OP_CASE(READY);
    GENERATE_OP_CASE(PLAN_EXECUTION);
    GENERATE_OP_CASE(TELEOPERATION);
    GENERATE_OP_CASE(AUTO_RETURN);
    GENERATE_OP_CASE(FAULT);
    GENERATE_OP_DEFAULT(READY);
  }
}

#define GENERATE_EXEC_CASE(NAME) \
  case ExecState::NAME: return rea::EXECUTION_STATE_##NAME

#define GENERATE_EXEC_DEFAULT(NAME) \
  default: \
    ROS_FATAL("unknown %s: %d", state_name, state); \
    return rea::EXECUTION_STATE_##NAME

rea::ExecutionState ConvertExecutionState(const uint8_t state) {
  static const char* state_name = "execution state";
  switch (state) {
    GENERATE_EXEC_CASE(IDLE);
    GENERATE_EXEC_CASE(EXECUTING);
    GENERATE_EXEC_CASE(PAUSED);
    GENERATE_EXEC_CASE(ERROR);
    GENERATE_EXEC_DEFAULT(IDLE);
  }
}

#define GENERATE_MOB_CASE(NAME) \
  case MobilityState::NAME: return rea::MOBILITY_STATE_##NAME

#define GENERATE_MOB_DEFAULT(NAME) \
  default: \
    ROS_FATAL("unknown %s: %d", state_name, state); \
    return rea::MOBILITY_STATE_##NAME

rea::MobilityState ConvertMobilityState(const uint8_t state) {
  static const char* state_name = "mobility state";
  switch (state) {
    GENERATE_MOB_CASE(DRIFTING);
    GENERATE_MOB_CASE(STOPPING);
    GENERATE_MOB_CASE(FLYING);
    GENERATE_MOB_CASE(DOCKING);
    GENERATE_MOB_CASE(PERCHING);
    GENERATE_MOB_DEFAULT(DRIFTING);
  }
}

}  // end namespace

ff::RosAgentStateToRapid::RosAgentStateToRapid(
                                            const std::string& subscribe_topic,
                                            const std::string& pub_topic,
                                            const ros::NodeHandle &nh,
                                            const unsigned int queue_size)
  : RosSubRapidPub(subscribe_topic, pub_topic, nh, queue_size) {
  state_supplier_.reset(
    new ff::RosAgentStateToRapid::StateSupplier(
      rapid::ext::astrobee::AGENT_STATE_TOPIC + pub_topic,
      "", "AstrobeeAgentStateProfile", ""));

  sub_ = nh_.subscribe(subscribe_topic,
                       queue_size,
                       &RosAgentStateToRapid::Callback,
                       this);

  rapid::RapidHelper::initHeader(state_supplier_->event().hdr);
}

// TODO(tfmorse): update for new agent state

void ff::RosAgentStateToRapid::Callback(
                          const ff_msgs::AgentStateStamped::ConstPtr& status) {
  rapid::ext::astrobee::AgentState &msg = state_supplier_->event();

  msg.hdr.timeStamp = util::RosTime2RapidTime(status->header.stamp);

  msg.operatingState = ConvertOperatingState(status->operating_state.state);
  msg.executionState = ConvertExecutionState(status->plan_execution_state.state);
  msg.mobilityState = ConvertMobilityState(status->mobility_state.state);
  msg.subMobilityState = status->mobility_state.sub_state;
  msg.guestScienceState = ConvertExecutionState(status->guest_science_state.state);
  msg.proximity = status->proximity;

  // Currently the code only supports profile names 32 characters long
  if (status->profile_name.size() > 32) {
    ROS_WARN("DDS: Profile name %s is longer than 32 characters!",
                                                  status->profile_name.c_str());
  }
  std::strncpy(msg.profileName, status->profile_name.data(), 32);
  msg.profileName[31] = '\0';

  // Currently the code only supports flight modes 32 charaters long
  if (status->flight_mode.size() > 32) {
    ROS_WARN("DDS: Flight mode %s is longer than 32 characters!",
                                                  status->flight_mode.c_str());
  }
  std::strncpy(msg.flightMode, status->flight_mode.data(), 32);
  msg.flightMode[31] = '\0';

  msg.targetLinearVelocity = status->target_linear_velocity;
  msg.targetLinearAccel = status->target_linear_accel;
  msg.targetAngularVelocity = status->target_angular_velocity;
  msg.targetAngularAccel = status->target_angular_accel;
  msg.collisionDistance = status->collision_distance;
  msg.enableHolonomic = status->holonomic_enabled;
  msg.checkObstacles = status->check_obstacles;
  msg.checkKeepouts = status->check_zones;
  msg.enableAutoReturn = status->auto_return_enabled;
  msg.bootTime = status->boot_time;

  state_supplier_->sendEvent();
}

