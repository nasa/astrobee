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


#include "dds_ros_bridge/ros_pmc_cmd_state.h"

namespace ff {

RosPmcCmdStateToRapid::RosPmcCmdStateToRapid(
                                            const std::string& subscribe_topic,
                                            const std::string& pub_topic,
                                            const ros::NodeHandle& nh,
                                            const unsigned int queue_size)
  : RosSubRapidPub(subscribe_topic, pub_topic, nh, queue_size) {
  state_supplier_.reset(new RosPmcCmdStateToRapid::StateSupplier(
      rapid::ext::astrobee::PMC_CMD_STATE_TOPIC + pub_topic,
      "",
      "AstrobeePmcCmdStateProfile",
      ""));

  // start subscriber
  sub_ = nh_.subscribe(subscribe_topic,
                       queue_size,
                       &RosPmcCmdStateToRapid::MsgCallback,
                       this);

  // Initialize the state message
  rapid::RapidHelper::initHeader(state_supplier_->event().hdr);

  // Setup time for publishing the gnc control state but don't start the timer
  // since the rate is 0. The bridge will set this rate at the end of its init
  // update: Andrew changed rate to 1.0 to avoid a runtime bounds error. Should
  // not affect since autostart argument is set to false.
  pmc_timer_ = nh_.createTimer(ros::Rate(1.0),
                               &RosPmcCmdStateToRapid::PubPmcCmdState,
                               this,
                               false,
                               false);
}

void RosPmcCmdStateToRapid::CopyPmcGoal(
        const ff_hw_msgs::PmcGoal& ros_goal, rapid::ext::astrobee::PmcGoal& dds_goal) {
  dds_goal.motorSpeed = ros_goal.motor_speed;
  dds_goal.nozzlePositions.ensure_length(6, 6);

  for (int i = 0; i < 6; i++) {
    dds_goal.nozzlePositions[i] = ros_goal.nozzle_positions[i];
  }
}

void RosPmcCmdStateToRapid::MsgCallback(
                                    const ff_hw_msgs::PmcCommandConstPtr& msg) {
  pmc_msg_ = msg;
}

void RosPmcCmdStateToRapid::PubPmcCmdState(const ros::TimerEvent& event) {
  if (pmc_msg_ == NULL) {
    return;
  }

  rapid::ext::astrobee::PmcCmdState &msg = state_supplier_->event();

  // Copy time
  msg.hdr.timeStamp = util::RosTime2RapidTime(pmc_msg_->header.stamp);

  int goals_size = pmc_msg_->goals.size();
  msg.goals.ensure_length(goals_size, 8);

  for (int i = 0; i < goals_size; i++) {
    CopyPmcGoal(pmc_msg_->goals[i], msg.goals[i]);
  }

  // Send message
  state_supplier_->sendEvent();
}

void RosPmcCmdStateToRapid::SetPmcPublishRate(float rate) {
  if (rate == 0) {
    pmc_timer_.stop();
  } else {
    pmc_timer_.setPeriod(ros::Duration(ros::Rate(rate)));
    pmc_timer_.start();
  }
}

}  // end namespace ff
