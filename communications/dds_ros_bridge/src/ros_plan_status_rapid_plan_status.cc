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

#include <string>
#include <cstring>
#include <algorithm>
#include <memory>

#include "dds_ros_bridge/ros_plan_status_rapid_plan_status.h"
#include "dds_ros_bridge/enum_helper.h"
#include "dds_ros_bridge/util.h"

#include "rapidUtil/RapidHelper.h"

#include "ff_msgs/AckStatus.h"
#include "ff_msgs/PlanStatusStamped.h"
#include "PlanStatusSupport.h"

ff::RosPlanStatusRapidPlanStatus::RosPlanStatusRapidPlanStatus(
    const std::string& subscribeTopic,
    const std::string& pubTopic,
    const ros::NodeHandle &nh,
    const unsigned int queueSize)
  : RosSubRapidPub(subscribeTopic, pubTopic, nh, queueSize) {
  m_supplier_.reset(
    new RosPlanStatusRapidPlanStatus::StatusSupplier(
      "astrobee_plan_status" + pubTopic,
      "", "RapidReliableDurableQos", ""));

  m_sub_ = m_nh_.subscribe(subscribeTopic, queueSize,
    &RosPlanStatusRapidPlanStatus::Callback, this);

  rapid::RapidHelper::initHeader(m_supplier_->event().hdr);

//  m_supplier_->event().
//  msg.currentCommand = -1;
}

void
ff::RosPlanStatusRapidPlanStatus::Callback(
  const ff_msgs::PlanStatusStamped::ConstPtr& status) {
  rapid::ext::astrobee::PlanStatus &msg = m_supplier_->event();

  msg.hdr.timeStamp = util::RosTime2RapidTime(status->header.stamp);

  std::strncpy(msg.planName, status->name.data(), 32);
  msg.planName[31] = '\0';  // ensure null-terminated

  msg.currentPoint = status->point;
  msg.currentCommand = status->command;
  msg.currentStatus = util::ConvertAckStatus(status->status);
  msg.statusHistory.ensure_length(status->history.size(),
                                                    status->history.size());
  for (std::size_t i = 0; i < status->history.size(); ++i) {
    msg.statusHistory[i].point = status->history[i].point;
    msg.statusHistory[i].command = status->history[i].command;
    msg.statusHistory[i].duration = status->history[i].duration;
    msg.statusHistory[i].status =
                util::ConvertAckCompletedStatus(status->history[i].status);
  }

  m_supplier_->sendEvent();
}

