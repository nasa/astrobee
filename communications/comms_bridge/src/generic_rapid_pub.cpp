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

#include "comms_bridge/generic_rapid_pub.h"

#include <string>

namespace ff {

GenericRapidPub::GenericRapidPub(std::string const& robot_name) :
    advertisement_info_seq_(0) {
  std::string dds_topic_name;
  dds_topic_name = robot_name + "-" +
                   rapid::ext::astrobee::GENERIC_COMMS_ADVERTISEMENT_INFO_TOPIC;
  ROS_INFO("Comms Bridge: DDS Pub DDS advertisement info topic name: %s\n",
           dds_topic_name.c_str());
  advertisement_info_supplier_.reset(
    new GenericRapidPub::AdvertisementInfoSupplier(
      dds_topic_name, "", "AstrobeeGenericCommsAdvertisementInfoProfile", ""));

  dds_topic_name = robot_name + "-" +
                   rapid::ext::astrobee::GENERIC_COMMS_CONTENT_TOPIC;
  ROS_INFO("Comms Bridge: DDS Publisher DDS content topic name: %s\n",
           dds_topic_name.c_str());
  content_supplier_.reset(new GenericRapidPub::ContentSupplier(
    dds_topic_name, "", "AstrobeeGenericCommsContentProfile", ""));

  rapid::RapidHelper::initHeader(advertisement_info_supplier_->event().hdr);
  rapid::RapidHelper::initHeader(content_supplier_->event().hdr);
}

GenericRapidPub::~GenericRapidPub() {}

template <typename T>
void GenericRapidPub::CopyString(const int max_size,
                                 std::string src,
                                 T &dest,
                                 std::string const& data_name,
                                 std::string const& topic) {
  unsigned int size = src.size();
  if (size > (max_size - 1)) {
    ROS_ERROR("Comms Bridge: %s for topic %s is greater than %i characters. Please change idl.",
              data_name.c_str(), topic.c_str(), max_size);
    size = (max_size - 1);
  }
  memset(dest, 0, max_size);
  strncpy(dest, src.data(), size);
  dest[size] = '\0';
}

void GenericRapidPub::SendAdvertisementInfo(std::string const& output_topic,
                                            bool latching,
                                            std::string const& data_type,
                                            std::string const& md5_sum,
                                            std::string definition) {
  rapid::ext::astrobee::GenericCommsAdvertisementInfo &msg =
                                          advertisement_info_supplier_->event();

  msg.hdr.timeStamp = comms_util::RosTime2RapidTime(ros::Time::now());
  msg.hdr.serial = ++advertisement_info_seq_;

  // Currrently the output topic can only be 128 characters long
  CopyString<rapid::String128>(128,
                               output_topic,
                               msg.outputTopic,
                               "Out topic",
                               output_topic);

  msg.latching = latching;

  // Currently the data type can only be 128 characters long
  CopyString<rapid::String128>(128,
                               data_type,
                               msg.dataType,
                               "Data type",
                               output_topic);

  // Currently the md5 sum can only be 64 characters long
  CopyString<rapid::String64>(64,
                              md5_sum,
                              msg.md5Sum,
                              "Md5 sum",
                              output_topic);

  // Remove legal statement(s)
  std::string::size_type begin = definition.find("# Copyright (c) 20");
  std::string::size_type end;
  while (begin != definition.npos) {
    end = definition.find("# under the License.");
    end += 20;
    definition.erase(begin, (end - begin));
    begin = definition.find("# Copyright (c) 20");
  }

  // Currently the ROS message definition can only by 8192 characters long
  ROS_INFO("Definition is %i long\n", definition.size());
  CopyString<rapid::ext::astrobee::String8K>(8192,
                                             definition,
                                             msg.msgDefinition,
                                             "Message definition",
                                             output_topic);

  // Send message
  advertisement_info_supplier_->sendEvent();
}

void GenericRapidPub::SendContent(std::string const& output_topic,
                                  std::string const& md5_sum,
                                  uint8_t const* data,
                                  const size_t data_size,
                                  const int seq_num) {
  unsigned int size;
  rapid::ext::astrobee::GenericCommsContent &msg = content_supplier_->event();

  msg.hdr.timeStamp = comms_util::RosTime2RapidTime(ros::Time::now());
  msg.hdr.serial = seq_num;

  // Currently the output topic can only be 128 characters long
  CopyString<rapid::String128>(128,
                               output_topic,
                               msg.outputTopic,
                               "Out topic",
                               output_topic);

  // Currently the md5 sum can only be 64 characters long
  CopyString<rapid::String64>(64,
                              md5_sum,
                              msg.md5Sum,
                              "Md5 sum",
                              output_topic);

  // Currently the content can only be 128K bytes long
  size = data_size;
  if (size > 131071) {
    ROS_ERROR("Comms Bridge: The message data for topic %s is greater than 131072 . Please change idl.",
              output_topic);
    size = 131071;
  }

  msg.data.ensure_length(size, (size + 1));
  unsigned char *buf = msg.data.get_contiguous_buffer();
  if (buf == NULL) {
    ROS_ERROR("DDS: RTI didn't give a contiguous buffer for the content data!");
    return;
  }
  std::memset(buf, 0, (size + 1));
  std::memmove(buf, data, size);

  // Send message
  content_supplier_->sendEvent();
}

}  // end namespace ff
