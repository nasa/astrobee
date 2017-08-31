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

#include <dds_ros_bridge/enum_helper.h>

#include <ros/assert.h>

#include <ff_msgs/AckStatus.h>
#include <ff_msgs/AckCompletedStatus.h>

#include <rapidDds/Ack.h>

#define GENERATE_ACK_CASE(NAME) \
  case ff_msgs::AckStatus::NAME: return rapid::ACK_##NAME

rapid::AckStatus util::ConvertAckStatus(ff_msgs::AckStatus const& status) {
  switch (status.status) {
    GENERATE_ACK_CASE(QUEUED);
    GENERATE_ACK_CASE(EXECUTING);
    GENERATE_ACK_CASE(REQUEUED);
    GENERATE_ACK_CASE(COMPLETED);
  default:
    ROS_ASSERT_MSG(false, "unknown AckStatus: %d", status.status);
    return rapid::ACK_QUEUED;  // Never reached
  }
}

#define GENERATE_COMPLETED_CASE(NAME) \
  case ff_msgs::AckCompletedStatus::NAME: return rapid::ACK_COMPLETED_##NAME

rapid::AckCompletedStatus
util::ConvertAckCompletedStatus(ff_msgs::AckCompletedStatus const& status) {
  switch (status.status) {
    GENERATE_COMPLETED_CASE(NOT);
    GENERATE_COMPLETED_CASE(OK);
    GENERATE_COMPLETED_CASE(BAD_SYNTAX);
    GENERATE_COMPLETED_CASE(EXEC_FAILED);
    GENERATE_COMPLETED_CASE(CANCELED);
  default:
    ROS_ASSERT_MSG(false, "unknown AckCompletedStatus: %d", status.status);
    return rapid::ACK_COMPLETED_NOT;  // never reached
  }
}

