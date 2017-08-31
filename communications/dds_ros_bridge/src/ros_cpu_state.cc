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


#include "dds_ros_bridge/ros_cpu_state.h"

ff::RosCpuStateToRapid::RosCpuStateToRapid(const std::string& subTopic,
                                           const std::string& pubTopic,
                                           const ros::NodeHandle &nh,
                                           const unsigned int queueSize) :
    RosSubRapidPub(subTopic, pubTopic, nh, queueSize) {
  c_supplier_.reset(new ff::RosCpuStateToRapid::ConfigSupplier(
          rapid::ext::astrobee::CPU_CONFIG_TOPIC + pubTopic, "",
          "AstrobeeCpuConfigProfile", ""));

  s_supplier_.reset(new ff::RosCpuStateToRapid::StateSupplier(
          rapid::ext::astrobee::CPU_STATE_TOPIC + pubTopic, "",
          "AstrobeeCpuStateProfile", ""));

  m_sub_ = m_nh_.subscribe(subTopic, queueSize, &RosCpuStateToRapid::Callback,
                           this);

  rapid::RapidHelper::initHeader(c_supplier_->event().hdr);
  rapid::RapidHelper::initHeader(s_supplier_->event().hdr);

  // Initialize the serial number to be 0, it will obe incremented upon
  // receiving unseen cpu state messages and when the max frequency changes for
  // a cpu
  // Technically this could have been set in the initHeader function but it is
  // the last argument and I didn't care to specify any of the other arguments
  c_supplier_->event().hdr.serial = 0;
  s_supplier_->event().hdr.serial = 0;

  // Setup timer for checking and publishing the state but don't start it since
  // rate is 0. The bridge will set this rate at the end of its initialization
  pub_timer_ = m_nh_.createTimer(ros::Rate(0),
                                 &RosCpuStateToRapid::CheckAndPublish,
                                 this,
                                 false,
                                 false);
  c_supplier_->sendEvent();
}

void ff::RosCpuStateToRapid::Callback(ff_msgs::CpuStateStampedConstPtr const&
                                                                        state) {
  bool found = false;
  int index = 0, i = 0;
  unsigned int total_index = 0;

  rapid::ext::astrobee::CpuConfig &c_msg = c_supplier_->event();
  rapid::ext::astrobee::CpuState &s_msg = s_supplier_->event();

  // Find where machine state goes in cpu state message
  for (index = 0; index < c_msg.machines.length(); index++) {
    if (c_msg.machines[index].name == state->name) {
      found = true;
      break;
    }
  }

  int num_cpus = state->cpus.size();

  if (!found) {
    if (index >= 8) {
      ROS_ERROR("DDS Bridge: Can only have 8 machines in cpu state but got %i.",
               index);
      return;
    }

    // Increment serial number since we are changing the config
    c_msg.hdr.serial++;
    s_msg.hdr.serial++;

    c_msg.machines.length((index + 1));
    s_msg.machines.length((index + 1));

    std::strncpy(c_msg.machines[index].name, state->name.data(), 16);
    c_msg.machines[index].name[15] = '\0';

    if (num_cpus >= 8) {
      ROS_ERROR("DDS Bridge: Can only have 8 cpu per machine but got %i for %s",
               num_cpus, state->name.c_str());
      num_cpus = 8;
    }

    c_msg.machines[index].num_cpus = num_cpus;
    c_msg.machines[index].max_frequencies.length(num_cpus);
    for (i = 0; i < num_cpus; i++) {
      c_msg.machines[index].max_frequencies[i] = state->cpus[i].max_frequency;
    }

    s_msg.machines[index].cpus.length(num_cpus);

    updated_ = true;

    c_msg.hdr.timeStamp = util::RosTime2RapidTime(state->header.stamp);
    c_supplier_->sendEvent();
  }

  // Need to find where the total is in the array of loads
  found = false;
  for (total_index = 0; total_index < state->load_fields.size();
                                                                total_index++) {
    if (state->load_fields[total_index] == ff_msgs::CpuStateStamped::TOTAL) {
      found = true;
      break;
    }
  }

  float load = 0;
  if (found) {
    load = state->ave_loads[total_index];
  }

  if (s_msg.machines[index].ave_total_load != load) {
    updated_ = true;
    s_msg.machines[index].ave_total_load = load;
  }

  if (s_msg.machines[index].temperature != state->temp) {
    updated_ = true;
    s_msg.machines[index].temperature = state->temp;
  }

  for (i = 0; i < num_cpus; i++) {
    // If total specified, set it. Otherwise set it to 0
    if (found) {
      load = state->cpus[i].loads[total_index];
    } else {
      load = 0;
    }

    if (s_msg.machines[index].cpus[i].total_load != load) {
      updated_ = true;
      s_msg.machines[index].cpus[i].total_load = load;
    }

    if (s_msg.machines[index].cpus[i].frequency != state->cpus[i].frequency) {
      updated_ = true;
      s_msg.machines[index].cpus[i].frequency = state->cpus[i].frequency;
    }
  }
}

void ff::RosCpuStateToRapid::CheckAndPublish(ros::TimerEvent const& event) {
  if (updated_) {
    s_supplier_->event().hdr.timeStamp =
                                      util::RosTime2RapidTime(ros::Time::now());
    s_supplier_->sendEvent();
    updated_ = false;
  }
}

void ff::RosCpuStateToRapid::SetPublishRate(float rate) {
  if (rate == 0) {
    pub_timer_.stop();
  } else {
    pub_timer_.setPeriod(ros::Duration(ros::Rate(rate)));
    pub_timer_.start();  // Start in case it was stopped or never started
  }
}
