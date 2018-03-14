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

#include "dds_ros_bridge/ros_plan_status_rapid_plan_status.h"

ff::RosPlanStatusRapidPlanStatus::RosPlanStatusRapidPlanStatus(
    const std::string& subscribe_topic,
    const std::string& pub_topic,
    const ros::NodeHandle &nh,
    const unsigned int queue_size)
  : RosSubRapidPub(subscribe_topic, pub_topic, nh, queue_size) {
  status_supplier_.reset(
    new RosPlanStatusRapidPlanStatus::StatusSupplier(
      rapid::ext::astrobee::PLAN_STATUS_TOPIC + pub_topic,
      "", "AstrobeePlanStatusProfile", ""));

  sub_ = nh_.subscribe(subscribe_topic,
                       queue_size,
                       &RosPlanStatusRapidPlanStatus::Callback,
                       this);

  rapid::RapidHelper::initHeader(status_supplier_->event().hdr);
}

void
ff::RosPlanStatusRapidPlanStatus::Callback(
                          const ff_msgs::PlanStatusStamped::ConstPtr& status) {
  rapid::ext::astrobee::PlanStatus &msg = status_supplier_->event();

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

  status_supplier_->sendEvent();
}

