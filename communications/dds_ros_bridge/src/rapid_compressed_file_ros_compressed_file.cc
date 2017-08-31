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

#include <stdint.h>

#include <ros/assert.h>

#include <string>
#include <vector>

#include "dds_ros_bridge/rapid_compressed_file_ros_compressed_file.h"
#include "dds_ros_bridge/util.h"

#include "ff_msgs/CompressedFile.h"

#include "CompressedFileSupport.h"


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
    const std::string& subscribeTopic,
    const std::string& pubTopic,
    const ros::NodeHandle &nh,
    const unsigned int queueSize)
  : ff::RapidSubRosPub(subscribeTopic, pubTopic, nh,
                       "RapidCompressedFileRosCompresesdFile", queueSize) {
  m_pub_ = m_nh_.advertise<ff_msgs::CompressedFile>(pubTopic, queueSize);

  try {
    m_ddsEventLoop_.connect<rapid::ext::astrobee::CompressedFile>(this,
      "astrobee_compressed_file" + subscribeTopic,
      "", "AstrobeeCompressedFile", "");
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

  m_pub_.publish(msg);
}

