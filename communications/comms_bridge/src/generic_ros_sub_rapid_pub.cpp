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

#include "comms_bridge/generic_ros_sub_rapid_pub.h"

#include <string>

namespace ff {

GenericROSSubRapidPub::GenericROSSubRapidPub() :
    dds_initialized_(false), advertisement_info_seq_(0) {}

GenericROSSubRapidPub::~GenericROSSubRapidPub() {}

void GenericROSSubRapidPub::InitializeDDS() {
  advertisement_info_supplier_.reset(
      new GenericROSSubRapidPub::AdvertisementInfoSupplier(
          rapid::ext::astrobee::GENERIC_COMMS_ADVERTISEMENT_INFO_TOPIC,
          "", "AstrobeeGenericCommsAdvertisementInfoProfile", ""));

  content_supplier_.reset(new GenericROSSubRapidPub::ContentSupplier(
                rapid::ext::astrobee::GENERIC_COMMS_CONTENT_TOPIC,
                "", "AstrobeeGenericCommsContentProfile", ""));

  rapid::RapidHelper::initHeader(advertisement_info_supplier_->event().hdr);
  rapid::RapidHelper::initHeader(content_supplier_->event().hdr);

  dds_initialized_ = true;
}

// Called with the mutex held
void GenericROSSubRapidPub::subscribeTopic(std::string const& in_topic, const RelayTopicInfo& info) {
  // this is just the base subscriber letting us know it's adding a topic
  // nothing more we need to do
}

// Called with the mutex held
void GenericROSSubRapidPub::advertiseTopic(const RelayTopicInfo& relay_info) {
  const AdvertisementInfo &info = relay_info.ad_info;
  rapid::ext::astrobee::GenericCommsAdvertisementInfo &msg =
                                        advertisement_info_supplier_->event();
  unsigned int size;
  std::string out_topic = relay_info.out_topic;

  msg.hdr.timeStamp = comms_util::RosTime2RapidTime(ros::Time::now());
  msg.hdr.serial = ++advertisement_info_seq_;

  // Currently the output topic can only be 128 characters long
  SizeCheck(size, out_topic.size(), 128, "Out topic", out_topic);
  std::strncpy(msg.outputTopic, out_topic.data(), size);
  msg.outputTopic[size] = '\0';

  msg.latching = info.latching;

  // Currently the data type can only be 128 characters long
  SizeCheck(size, info.data_type.size(), 128, "Data type",  out_topic);
  std::strncpy(msg.dataType, info.data_type.data(), size);
  msg.dataType[size] = '\0';

  // Currently the md5 sum can only be 32 characters long
  SizeCheck(size, info.md5_sum.size(), 32, "MD5 sum", out_topic);
  std::strncpy(msg.md5Sum, info.md5_sum.data(), size);
  msg.md5Sum[size] = '\0';

  // Current the ROS message definition can only be 2048 characters long
  SizeCheck(size, info.definition.size(), 2048, "Msg definition", out_topic);
  std::strncpy(msg.msgDefinition, info.definition.data(), size);
  msg.msgDefinition[size] = '\0';

  // Send message
  advertisement_info_supplier_->sendEvent();
}

// Called with the mutex held
void GenericROSSubRapidPub::relayMessage(const RelayTopicInfo& topic_info, ContentInfo const& content_info) {
  rapid::ext::astrobee::GenericCommsContent &msg = content_supplier_->event();

  unsigned int size;
  std::string out_topic = topic_info.out_topic;

  msg.hdr.timeStamp = comms_util::RosTime2RapidTime(ros::Time::now());
  msg.hdr.serial = topic_info.relay_seqnum;

  // Currently the output topic can only be 128 characters long
  SizeCheck(size, out_topic.size(), 128, "Out topic", out_topic);
  std::strncpy(msg.outputTopic, out_topic.data(), size);
  msg.outputTopic[size] = '\0';

  // Currently the md5 sum can only be 32 characters long
  SizeCheck(size, content_info.type_md5_sum.size(), 32, "MD5 sum", out_topic);
  std::strncpy(msg.md5Sum, content_info.type_md5_sum.data(), size);
  msg.md5Sum[size] = '\0';

  // Currently the content can only be 128K bytes long
  SizeCheck(size, content_info.data_size, 131072, "Data", out_topic);
  msg.data.ensure_length(size, (size + 1));
  unsigned char *buf = msg.data.get_contiguous_buffer();
  if (buf == NULL) {
    ROS_ERROR("DDS: RTI didn't give a contiguous buffer for the content data!");
    return;
  }

  std::memset(buf, 0, (size + 1));
  std::memmove(buf, content_info.data, size);

  // Send message
  content_supplier_->sendEvent();
}

void GenericROSSubRapidPub::SizeCheck(unsigned int &size,
                                    const int size_in,
                                    const int max_size,
                                    std::string const& data_name,
                                    std::string const& topic) {
  size = size_in;
  // Account for null
  if (size > (max_size - 1)) {
    ROS_ERROR("Comms Bridge: %s for topic %s is greater than %i characters. Please change idl.",
              data_name.c_str(), topic.c_str(), max_size);
    size = (max_size - 1);
  }
}

}  // end namespace ff
