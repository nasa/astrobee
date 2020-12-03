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

#ifndef LOCALIZATION_COMMON_ROS_TIMER_H_
#define LOCALIZATION_COMMON_ROS_TIMER_H_

#include <localization_common/time.h>

#include <ros/time.h>
#include <std_msgs/Header.h>

#include <string>

namespace localization_common {
class RosTimer {
 public:
  explicit RosTimer(const std::string& timer_name);
  void Start();
  // Uses header time as start time and ros::Time::now as stop time
  void HeaderDiff(const std_msgs::Header& header);
  void Stop();
  void Log() const;
  void Vlog(const int level = 2) const;
  void LogEveryN(const int num_timing_events_per_log) const;
  void VlogEveryN(const int num_timing_events_per_log, const int level = 2) const;

 private:
  std::string name_;
  ros::Time start_time_;
  double last_elapsed_time_;
  double average_elapsed_time_;
  int num_timing_events_;
};
}  // namespace localization_common

#endif  // LOCALIZATION_COMMON_ROS_TIMER_H_
