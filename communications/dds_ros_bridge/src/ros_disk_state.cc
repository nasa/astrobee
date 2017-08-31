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

#include "dds_ros_bridge/ros_disk_state.h"

ff::RosDiskStateToRapid::RosDiskStateToRapid(
    const std::string& subscribeTopic,
    const std::string& pubTopic,
    const ros::NodeHandle &nh,
    const unsigned int queueSize) :
    RosSubRapidPub(subscribeTopic, pubTopic, nh, queueSize),
    updated_(false),
    num_disks_(0),
    llp_start_index_(-1),
    mlp_start_index_(-1),
    hlp_start_index_(-1) {
  s_supplier_.reset(
    new ff::RosDiskStateToRapid::StateSupplier(
       rapid::ext::astrobee::DISK_STATE_TOPIC + pubTopic, "",
      "AstrobeeDiskStateProfile", ""));

  c_supplier_.reset(new ff::RosDiskStateToRapid::ConfigSupplier(
        rapid::ext::astrobee::DISK_CONFIG_TOPIC + pubTopic, "",
        "AstrobeeDiskConfigProfile", ""));


  m_sub_ = m_nh_.subscribe(subscribeTopic, queueSize,
                           &RosDiskStateToRapid::Callback, this);

  rapid::RapidHelper::initHeader(c_supplier_->event().hdr);
  rapid::RapidHelper::initHeader(s_supplier_->event().hdr);

  // Initialize the serial number to be 0, it will be incremented upon new
  // processor state messages. Since there is only 3 processors, the serial
  // number should be 3 after everything is operating correctly
  // technically this could have been set in the initHeader function but it is
  // the last argument and I didn't care to specify any of the other arguments
  c_supplier_->event().hdr.serial = 0;
  s_supplier_->event().hdr.serial = 0;

  // Setup timer for checking and publishing the state but don't start it since
  // the rate is 0. The bridge will set this rate at the end of its
  // initialization
  pub_timer_ = m_nh_.createTimer(ros::Rate(0),
                              &RosDiskStateToRapid::CheckAndPublish,
                              this,
                              false,
                              false);
}

void ff::RosDiskStateToRapid::Callback(const ff_msgs::DiskStateStampedConstPtr&
                                                                        state) {
  int j = 0, index;
  bool add_to_config = false;

  rapid::ext::astrobee::DiskConfig &c_msg = c_supplier_->event();
  rapid::ext::astrobee::DiskState &s_msg = s_supplier_->event();

  // Check to see what processor the message came from and if we have already
  // received the first state message from the processor. If it is, initialize
  // index and message. If not, see if you need to update the first state msg
  if (state->processor_name == "llp") {
    if (llp_start_index_ == -1) {
      llp_start_index_ = num_disks_;
      add_to_config = true;
    }

    index = llp_start_index_;
  } else if (state->processor_name == "mlp") {
    if (mlp_start_index_ == -1) {
      mlp_start_index_ = num_disks_;
      add_to_config = true;
    }

    index = mlp_start_index_;
  } else if (state->processor_name == "hlp") {
    if (hlp_start_index_ == -1) {
      hlp_start_index_ = num_disks_;
      add_to_config = true;
    }

    index = hlp_start_index_;
  } else {
    ROS_FATAL("DDS Bridge: Processor %s not recognized in disk state!",
              state->processor_name.c_str());
    return;
  }

  if (add_to_config) {
    // increase the number of disks
    num_disks_ += state->disks.size();
    if (num_disks_ > 8) {
      ROS_ERROR("DDS Bridge: Number of disks is greater than 8!");
      num_disks_ = 8;
    }
    // Resize file systems
    c_msg.filesystems.length(num_disks_);
    s_msg.filesystems.length(num_disks_);
    j = 0;
    std::string temp_name;

    // Add data to both config and state
    for (auto i = index; i < num_disks_; i++) {
      temp_name = state->processor_name + " - " + state->disks[j].path;
      std::strncpy(c_msg.filesystems[i].name, temp_name.data(), 32);
      c_msg.filesystems[i].name[31] = '\0';
      c_msg.filesystems[i].capacity = state->disks[j].capacity;

      s_msg.filesystems[i].used = state->disks[j].used;
      j++;
    }

    // increase serial number since we are sending a new config
    c_msg.hdr.serial += 1;
    s_msg.hdr.serial += 1;

    // set updated to true so date gets published next timeout
    updated_ = true;

    // Send config message
    c_msg.hdr.timeStamp = util::RosTime2RapidTime(state->header.stamp);
    c_supplier_->sendEvent();
  } else {
    j = 0;
    for (unsigned int i = index; (i < (index + state->disks.size()) && i < 8);
                                                                          i++) {
      // Check to see if the value has changed
      if (s_msg.filesystems[i].used != (int64_t) state->disks[j].used) {
        s_msg.filesystems[i].used = state->disks[j].used;
        updated_ = true;
      }
      j++;
    }
  }
}

void ff::RosDiskStateToRapid::CheckAndPublish(ros::TimerEvent const& event) {
  if (updated_) {
    s_supplier_->event().hdr.timeStamp =
                                      util::RosTime2RapidTime(ros::Time::now());
    s_supplier_->sendEvent();
    updated_ = false;
  }
}

void ff::RosDiskStateToRapid::SetPublishRate(float rate) {
  if (rate == 0) {
    pub_timer_.stop();
  } else {
    pub_timer_.setPeriod(ros::Duration(ros::Rate(rate)));
    pub_timer_.start();  // Start in case it was never started
  }
}
