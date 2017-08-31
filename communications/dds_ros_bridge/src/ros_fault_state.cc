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
#include <memory>

#include "dds_ros_bridge/ros_fault_state.h"
#include "dds_ros_bridge/enum_helper.h"
#include "dds_ros_bridge/util.h"

#include "rapidUtil/RapidHelper.h"

#include "ff_msgs/FaultState.h"
#include "FaultStateSupport.h"

ff::RosFaultStateToRapid::RosFaultStateToRapid(
    const std::string& subscribeTopic,
    const std::string& pubTopic,
    const ros::NodeHandle &nh,
    const unsigned int queueSize)
  : RosSubRapidPub(subscribeTopic, pubTopic, nh, queueSize) {
  m_supplier_.reset(
    new ff::RosFaultStateToRapid::StateSupplier(
        rapid::ext::astrobee::FAULT_STATE_TOPIC + pubTopic,
        "", "AstrobeeFaultStateProfile", ""));

  m_sub_ = m_nh_.subscribe(subscribeTopic, queueSize,
    &RosFaultStateToRapid::Callback, this);

  rapid::RapidHelper::initHeader(m_supplier_->event().hdr);
}

void ff::RosFaultStateToRapid::Callback(const ff_msgs::FaultStateConstPtr&
                                                                        state) {
  rapid::ext::astrobee::FaultState &msg = m_supplier_->event();
  msg.hdr.timeStamp = util::RosTime2RapidTime(state->header.stamp);

  unsigned int i, j, data_size, fault_size;

  // Currently the code only supports 32 faults triggered at a time
  fault_size = state->faults.size();
  if (fault_size > 32) {
    ROS_ERROR("DDS: There are %i faults but can only send 32 to the ground",
                                                                    fault_size);
    fault_size = 32;
  }
  msg.faults.length(fault_size);
  for (i = 0; i < fault_size; i++) {
    msg.faults[i].timestamp =
                        util::RosTime2RapidTime(state->faults[i].time_of_fault);
    msg.faults[i].code = state->faults[i].id;

    std::strncpy(msg.faults[i].message, state->faults[i].msg.data(), 128);
    msg.faults[i].message[127] = '\0';

    // Currently the code only supports 8 key type values
    data_size = state->faults[i].data.size();
    if (data_size > 8) {
      ROS_ERROR("DDS: There are %i key type values but only 8 sent to ground",
                                                                    data_size);
      data_size = 8;
    }
    msg.faults[i].data.length(data_size);
    for (j = 0; j < data_size; j++) {
      std::strncpy(msg.faults[i].data[j].key,
                                      state->faults[i].data[j].key.data(), 32);
      msg.faults[i].data[j].key[31] = '\0';

      switch (state->faults[i].data[j].data_type) {
        case ff_msgs::FaultData::DATA_TYPE_FLOAT:
          msg.faults[i].data[j].value._d = rapid::RAPID_FLOAT;
          msg.faults[i].data[j].value._u.f = state->faults[i].data[j].f;
          break;
        case ff_msgs::FaultData::DATA_TYPE_INT:
          msg.faults[i].data[j].value._d = rapid::RAPID_INT;
          msg.faults[i].data[j].value._u.i = state->faults[i].data[j].i;
          break;
        case ff_msgs::FaultData::DATA_TYPE_STRING:
          msg.faults[i].data[j].value._d = rapid::RAPID_STRING;
          std::strncpy(msg.faults[i].data[j].value._u.s,
                                        state->faults[i].data[j].s.data(), 128);
          msg.faults[i].data[j].value._u.s[127] = '\0';
          break;
        default:
          ROS_FATAL("DDS-ROS-Bridge: unknown fault data type: %i",
                                            state->faults[i].data[j].data_type);
      }
    }
  }

  m_supplier_->sendEvent();
}

