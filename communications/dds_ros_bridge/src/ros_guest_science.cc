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

#include "dds_ros_bridge/ros_guest_science.h"

using ff_msgs::GuestScienceData;

namespace {

#define GENERATE_GUEST_SCIENCE_CASE(NAME) \
  case GuestScienceData::NAME: return rapid::ext::astrobee::GUEST_SCIENCE_##NAME

#define GENERATE_GUEST_SCIENCE_DEFAULT(NAME) \
  default: \
    ROS_FATAL("DDS: Unknown guest science data type: %d", state); \
    return rapid::ext::astrobee::GUEST_SCIENCE_##NAME

rapid::ext::astrobee::GuestScienceDataType ConvertType(uint8_t state) {
  switch (state) {
    GENERATE_GUEST_SCIENCE_CASE(STRING);
    GENERATE_GUEST_SCIENCE_CASE(JSON);
    GENERATE_GUEST_SCIENCE_CASE(BINARY);
    GENERATE_GUEST_SCIENCE_DEFAULT(STRING);
  }
}

}  // end namespace

ff::RosGuestScienceToRapid::RosGuestScienceToRapid(
                                        const std::string& stateSubscribeTopic,
                                        const std::string& configSubscribeTopic,
                                        const std::string& dataSubscribeTopic,
                                        const std::string& pubTopic,
                                        const ros::NodeHandle &nh,
                                        const unsigned int queueSize) :
    RosSubRapidPub(stateSubscribeTopic, pubTopic, nh, queueSize),
    m_configSubscribeTopic_(configSubscribeTopic),
    m_dataSubscribeTopic_(dataSubscribeTopic) {
  c_supplier_.reset(new ff::RosGuestScienceToRapid::ConfigSupplier(
        rapid::ext::astrobee::GUEST_SCIENCE_CONFIG_TOPIC + pubTopic, "",
        "AstrobeeGuestScienceConfigProfile", ""));

  d_supplier_.reset(new ff::RosGuestScienceToRapid::DataSupplier(
          rapid::ext::astrobee::GUEST_SCIENCE_DATA_TOPIC + pubTopic, "",
          "AstrobeeGuestScienceDataProfile", ""));

  s_supplier_.reset(new ff::RosGuestScienceToRapid::StateSupplier(
        rapid::ext::astrobee::GUEST_SCIENCE_STATE_TOPIC + pubTopic, "",
        "AstrobeeGuestScienceStateProfile", ""));


  m_sub_ = m_nh_.subscribe(stateSubscribeTopic, queueSize,
                           &RosGuestScienceToRapid::StateCallback, this);

  m_configSub_ = m_nh_.subscribe(configSubscribeTopic, queueSize,
                                 &RosGuestScienceToRapid::ConfigCallback, this);

  m_dataSub_ = m_nh_.subscribe(dataSubscribeTopic, queueSize,
                               &RosGuestScienceToRapid::DataCallback, this);

  rapid::RapidHelper::initHeader(c_supplier_->event().hdr);
  rapid::RapidHelper::initHeader(d_supplier_->event().hdr);
  rapid::RapidHelper::initHeader(s_supplier_->event().hdr);
}

void ff::RosGuestScienceToRapid::ConfigCallback(
                            ff_msgs::GuestScienceConfigConstPtr const& config) {
  rapid::ext::astrobee::GuestScienceConfig &msg = c_supplier_->event();
  msg.hdr.timeStamp = util::RosTime2RapidTime(config->header.stamp);
  msg.hdr.serial = config->serial;

  // Copy over guest science apks
  // Currently the code only supports 32 apks
  unsigned int size = config->apks.size();
  if (size > 32) {
    ROS_ERROR("DDS: There are %i apks but only 32 can be sent to the ground.",
                                                                          size);
    size = 32;
  }

  msg.apkStates.length(size);
  for (unsigned int i = 0; i < size; i++) {
    // Currently the full apk name can only be 128 chacters long
    if (config->apks[i].apk_name.size() > 128) {
      ROS_ERROR("DDS: Apk %s's name is longer than 128 characters!",
                config->apks[i].apk_name.c_str());
    }
    std::strncpy(msg.apkStates[i].apkName, config->apks[i].apk_name.data(),
                                                                          128);
    msg.apkStates[i].apkName[127] = '\0';

    // Currently the short apk name can only be 32 characters long
    if (config->apks[i].short_name.size() > 32) {
      ROS_ERROR("DDS: Apk %s's short name is longer than 32 characters!",
                config->apks[i].short_name.c_str());
    }
    std::strncpy(msg.apkStates[i].shortName, config->apks[i].short_name.data(),
                                                                            32);
    msg.apkStates[i].shortName[31] = '\0';

    msg.apkStates[i].primary = config->apks[i].primary;

    // Currently the code only supports 32 commands per apk
    unsigned int num_cmds = config->apks[i].commands.size();
    if (num_cmds > 32) {
      ROS_ERROR("DDS: Apk %s has %i commands but only 32 can be sent to ground",
                config->apks[i].apk_name.c_str(), num_cmds);
      num_cmds = 32;
    }

    msg.apkStates[i].commands.length(num_cmds);
    for (unsigned int j = 0; j < num_cmds; j++) {
      // Currenty the command name can only be 32 characters long
      if (config->apks[i].commands[j].name.size() > 32) {
        ROS_ERROR("DDS: Apk %s command name %s is longer than 32 characters!",
                  config->apks[i].apk_name.c_str(),
                  config->apks[i].commands[j].name.c_str());
      }
      std::strncpy(msg.apkStates[i].commands[j].name,
                   config->apks[i].commands[j].name.data(), 32);
      msg.apkStates[i].commands[j].name[31] = '\0';

      // Currently the command syntax can only be 128 characters long
      if (config->apks[i].commands[j].command.size() > 128) {
        ROS_ERROR("DDS: Apk %s command syntax %s is longer than 32 characters!",
                  config->apks[i].apk_name.c_str(),
                  config->apks[i].commands[j].command.c_str());
      }
      std::strncpy(msg.apkStates[i].commands[j].command,
                   config->apks[i].commands[j].command.data(), 128);
      msg.apkStates[i].commands[j].command[127] = '\0';
    }
  }

  c_supplier_->sendEvent();
}

void ff::RosGuestScienceToRapid::DataCallback(
                                ff_msgs::GuestScienceDataConstPtr const& data) {
  rapid::ext::astrobee::GuestScienceData &msg = d_supplier_->event();
  msg.hdr.timeStamp = util::RosTime2RapidTime(data->header.stamp);

  // Don't check size of apk name because it is reported as too long when
  // sending apk config
  std::strncpy(msg.apkName, data->apk_name.data(), 128);
  msg.apkName[127] = '\0';

  msg.type = ConvertType(data->data_type);

  if (data->topic.size() > 32) {
    ROS_ERROR("DDS: Topic %s for Apk %s is longer than 32 characters!",
              data->topic.c_str(), data->apk_name.c_str());
  }
  std::strncpy(msg.topic, data->topic.data(), 32);
  msg.topic[31] = '\0';

  int size = data->data.size();
  if (size > 2048) {
    ROS_ERROR("DDS: Data with topic %s for Apk %s has %i bytes but only 2048 " \
              "can be sent to the ground.", data->topic.c_str(),
              data->apk_name.c_str(), size);
    size = 2048;
  }

  // resize
  msg.data.ensure_length(size, size);

  unsigned char *buff = msg.data.get_contiguous_buffer();
  if (buff == NULL) {
    ROS_ERROR("DDS: RTI didn't give contiguous buffer for guest science data!");
    return;
  }

  std::memmove(buff, data->data.data(), size);

  d_supplier_->sendEvent();
}

void ff::RosGuestScienceToRapid::StateCallback(
                              ff_msgs::GuestScienceStateConstPtr const& state) {
  rapid::ext::astrobee::GuestScienceState &msg = s_supplier_->event();
  msg.hdr.timeStamp = util::RosTime2RapidTime(state->header.stamp);

  int num_apks = state->runningApks.size();
  // Can only send 32 apks to the ground
  if (num_apks > 32) {
    num_apks = 32;
  }

  // Check if the serial number was changed. If so, probably need to resize the
  // apks running array
  if (state->serial != msg.hdr.serial) {
    msg.hdr.serial = state->serial;
    if (num_apks != msg.runningApks.length()) {
      msg.runningApks.length(num_apks);
    }
  }

  // Make sure state size matches config size
  if (msg.hdr.serial == c_supplier_->event().hdr.serial &&
      num_apks != c_supplier_->event().apkStates.length()) {
    ROS_ERROR("DDS: Number of apks don't match between the config and state!");
    return;
  }

  for (int i = 0; i < num_apks; i++) {
    msg.runningApks[i] = state->runningApks[i];
  }

  s_supplier_->sendEvent();
}
