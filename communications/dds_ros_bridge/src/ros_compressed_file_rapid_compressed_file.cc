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

#include "dds_ros_bridge/ros_compressed_file_rapid_compressed_file.h"

namespace {

namespace rea = rapid::ext::astrobee;
using ff_msgs::CompressedFile;

#define GENERATE_CASE(NAME) \
  case CompressedFile::TYPE_##NAME: return rea::COMPRESSION_TYPE_##NAME

#define GENERATE_DEFAULT(NAME) \
  default: \
    ROS_FATAL("unknown %s: %d", type_name, type); \
    return rea::COMPRESSION_TYPE_##NAME

rea::FileCompressionType ConvertCompression(const uint8_t type) {
  static const char* type_name = "file compression type";
  switch (type) {
    GENERATE_CASE(NONE);
    GENERATE_CASE(DEFLATE);
    GENERATE_CASE(BZ2);
    GENERATE_CASE(GZ);
    GENERATE_CASE(ZIP);
    GENERATE_DEFAULT(NONE);
  }
}

}  // namespace

ff::RosCompressedFileToRapid::RosCompressedFileToRapid(
                                            const std::string& subscribe_topic,
                                            const std::string& pub_topic,
                                            const ros::NodeHandle &nh,
                                            const unsigned int queue_size)
  : RosSubRapidPub(subscribe_topic, pub_topic, nh, queue_size) {
  state_supplier_.reset(
    new ff::RosCompressedFileToRapid::Supplier(
      rapid::ext::astrobee::COMPRESSED_FILE_TOPIC + pub_topic,
      "", "AstrobeeCurrentCompressedPlanProfile", ""));

  sub_ = nh_.subscribe(subscribe_topic,
                       queue_size,
                       &RosCompressedFileToRapid::Callback,
                       this);

  rapid::RapidHelper::initHeader(state_supplier_->event().hdr);
}

void ff::RosCompressedFileToRapid::Callback(
                                const ff_msgs::CompressedFile::ConstPtr& file) {
  rea::CompressedFile &msg = state_supplier_->event();
  msg.hdr.timeStamp = util::RosTime2RapidTime(file->header.stamp);

  msg.id = file->id;
  msg.compressionType = ConvertCompression(file->type);

  // resize
  msg.compressedFile.ensure_length(file->file.size(), file->file.size());

  // get access to raw bytes
  unsigned char *buf = msg.compressedFile.get_contiguous_buffer();
  if (buf == NULL) {
    ROS_WARN("RTI wants to give me a discontinous buffer? out of here");
    return;
  }

  // actually copy data
  std::memmove(buf, file->file.data(), file->file.size());

  state_supplier_->sendEvent();
}

