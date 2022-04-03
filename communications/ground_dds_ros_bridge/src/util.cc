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

#include <ground_dds_ros_bridge/util.h>

ros::Time util::RapidTime2RosTime(const int64_t dds_time) {
  ros::Time t;

  // DDS time is in microseconds, we need nanoseconds
  t.fromNSec(static_cast<uint64_t>(dds_time) * 1000);

  return t;
}

int64_t util::RosTime2RapidTime(ros::Time const& ros_time) {
  return static_cast<int64_t>(ros_time.toNSec() / 1000ULL);
}

void util::RosHeader2Rapid(std_msgs::Header const& ros_hdr,
                           rapid::Header* rapid_hdr,
                           int status,
                           int serial) {
  rapid::RapidHelper::initHeader(*rapid_hdr);
  if (serial >= 0) {
    rapid_hdr->serial = serial;
  }
  rapid_hdr->statusCode = status;
  rapid_hdr->timeStamp = util::RosTime2RapidTime(ros_hdr.stamp);
}

void util::RapidHeader2Ros(rapid::Header const& rapid_hdr,
                           std_msgs::Header *ros_hdr,
                           std::string const& frame_id) {
  ros_hdr->frame_id = frame_id;
  ros_hdr->stamp = util::RapidTime2RosTime(rapid_hdr.timeStamp);
}
