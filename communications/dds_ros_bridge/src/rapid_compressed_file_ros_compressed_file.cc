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

#include "dds_ros_bridge/rapid_compressed_file_ros_compressed_file.h"

namespace {

#define GENERATE_CASE(NAME) \
  case rapid::ext::astrobee::COMPRESSION_##NAME: \
    return ff_msgs::CompressedFile::NAME

uint8_t
RapidCompression2Ros(rapid::ext::astrobee::FileCompressionType const& t) {
  switch (t) {
    GENERATE_CASE(TYPE_NONE);
    GENERATE_CASE(TYPE_DEFLATE);
    GENERATE_CASE(TYPE_BZ2);
    GENERATE_CASE(TYPE_GZ);
    GENERATE_CASE(TYPE_ZIP);
  default:
    ROS_ASSERT_MSG(false, "unknown compression type: %d", t);
    return 0;  // never reached, but compiler complains
  }
}

}  // end namespace

ff::RapidCompressedFileRosCompressedFile::RapidCompressedFileRosCompressedFile(
    const std::string& subscribe_topic,
    const std::string& pub_topic,
    const ros::NodeHandle &nh,
    const unsigned int queue_size)
  : ff::RapidSubRosPub(subscribe_topic,
                       pub_topic,
                       nh,
                       "RapidCompressedFileRosCompresesdFile",
                       queue_size) {
  pub_ = nh_.advertise<ff_msgs::CompressedFile>(pub_topic, queue_size);

  try {
    dds_event_loop_.connect<rapid::ext::astrobee::CompressedFile>(this,
      rapid::ext::astrobee::COMPRESSED_FILE_TOPIC + subscribe_topic,
      "", "AstrobeeCompressedFileProfile", "");
  } catch (std::exception& e) {
    ROS_ERROR_STREAM("RapidCompressedFileRosCompressedFile exception: "
                     << e.what());
    throw;
  } catch (...) {
    ROS_ERROR("RapidCompressedFileRosCompressedFile exception unknown");
    throw;
  }

  // start thread
  StartThread();
}

void ff::RapidCompressedFileRosCompressedFile::operator() (
  rapid::ext::astrobee::CompressedFile const* file) {
  ff_msgs::CompressedFile msg;
  util::RapidHeader2Ros(file->hdr, &msg.header);

  msg.id = file->id;

  msg.type = RapidCompression2Ros(file->compressionType);

  unsigned char* buf = file->compressedFile.get_contiguous_buffer();
  msg.file.reserve(file->compressedFile.length());
  msg.file.resize(file->compressedFile.length());
  std::memmove(msg.file.data(), buf, file->compressedFile.length());

  pub_.publish(msg);
}

