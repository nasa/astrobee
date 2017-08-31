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

#ifndef DDS_ROS_BRIDGE_UTIL_H_
#define DDS_ROS_BRIDGE_UTIL_H_

#include <ros/time.h>

#include <std_msgs/Header.h>
#include <rapidDds/Header.h>

#include <stdint.h>

#include <string>

namespace util {

  // Time-related helpers
  ::ros::Time RapidTime2RosTime(const int64_t dds_time);

  int64_t RosTime2RapidTime(::ros::Time const& ros_time);

  // Header-related helpers
  void RosHeader2Rapid(::std_msgs::Header const& ros_hdr,
                       ::rapid::Header* rapid_hdr,
                       int status = 0, int serial = -1);

  void RapidHeader2Ros(::rapid::Header const& rapid_hdr,
                       ::std_msgs::Header* ros_hdr,
                       std::string const& frame_id = "world");

}  // end namespace util

#endif  // DDS_ROS_BRIDGE_UTIL_H_
