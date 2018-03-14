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

#include "dds_ros_bridge/ros_fault_config.h"

ff::RosFaultConfigToRapid::RosFaultConfigToRapid(
                                            const std::string& subscribe_topic,
                                            const std::string& pub_topic,
                                            const ros::NodeHandle &nh,
                                            const unsigned int queue_size)
  : RosSubRapidPub(subscribe_topic, pub_topic, nh, queue_size) {
  config_supplier_.reset(
    new ff::RosFaultConfigToRapid::ConfigSupplier(
        rapid::ext::astrobee::FAULT_CONFIG_TOPIC + pub_topic, "",
        "AstrobeeFaultConfigProfile", ""));

  sub_ = nh_.subscribe(subscribe_topic,
                       queue_size,
                       &RosFaultConfigToRapid::Callback,
                       this);

  rapid::RapidHelper::initHeader(config_supplier_->event().hdr);
}

void ff::RosFaultConfigToRapid::Callback(const ff_msgs::FaultConfigConstPtr&
                                                                      config) {
  rapid::ext::astrobee::FaultConfig &msg = config_supplier_->event();
  msg.hdr.timeStamp = util::RosTime2RapidTime(config->header.stamp);

  unsigned int i, size;

  // Copy over subsystem names
  // Currently the code only supports 16 subsystems
  size = config->subsystems.size();
  if (size > 16) {
    ROS_ERROR("DDS: There are %i subsystems but can only send 16 to the ground",
              size);
    size = 16;
  }

  msg.subsystems.length(size);
  for (i = 0; i < size; i++) {
    // Currently the code only supports subsystem names to be 32 characters long
    if (config->subsystems[i].size() > 32) {
      ROS_ERROR("DDS: Subsystem named %s is longer than 32 characters!",
                config->subsystems[i].c_str());
    }
    std::strncpy(msg.subsystems[i], config->subsystems[i].data(), 32);
    msg.subsystems[i][31] = '\0';
  }

  // Copy over node names
  // Currently the code only supports 128 nodes
  size = config->nodes.size();
  if (size > 128) {
    ROS_ERROR("DDS: There are %i nodes but can only send 128 to the ground.",
              size);
    size = 128;
  }

  msg.nodes.length(size);
  for (i = 0; i < size; i++) {
    // Currently the code only supports node names to be 32 characters long
    if (config->nodes[i].size() > 32) {
      ROS_ERROR("DDS: Node named %s is longer than 32 characters!",
                config->nodes[i].c_str());
    }
    std::strncpy(msg.nodes[i], config->nodes[i].data(), 32);
    msg.nodes[i][31] = '\0';
  }

  // Copy over faults
  // Currently the code only supports 256 faults
  size = config->faults.size();
  if (size > 256) {
    ROS_ERROR("DDS: There are %i faults but can only send 256 to the ground.",
              size);
    size = 256;
  }

  msg.faults.length(size);
  for (i = 0; i < size; i++) {
    msg.faults[i].subsystem = config->faults[i].subsystem;
    msg.faults[i].node = config->faults[i].node;
    msg.faults[i].faultId = config->faults[i].id;
    msg.faults[i].warning = config->faults[i].warning;

    // Currenlty the code only supports fault descriptions 64 characters long
    if (config->faults[i].description.size() > 64) {
      ROS_ERROR("DDS: Fault description %s is longer than 64 characters!",
                config->faults[i].description.c_str());
    }
    std::strncpy(msg.faults[i].faultDescription,
                 config->faults[i].description.data(),
                 64);
    msg.faults[i].faultDescription[63] = '\0';
  }

  config_supplier_->sendEvent();
}

